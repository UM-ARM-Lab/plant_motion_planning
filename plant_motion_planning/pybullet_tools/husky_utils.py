import numpy as np
import pybullet as p
import rospkg
import torch
import yaml
import re
import random
from arm_pytorch_utilities.math_utils import angular_diff_batch
from pytorch_mppi import mppi
from ompl import base

from plant_motion_planning.pybullet_tools.utils import step_simulation, wait_for_duration


class HuskyUtils:
    def __init__(self, env, floor, z_coord, time_step, device, arm_joints, srdf_file, goal_error=2e-1):
        self.env = env
        self.robot = env.robot
        self.z_coord = z_coord
        self.goal_error = goal_error
        self.device = device

        self.STATE_DIM = 5
        self.CONTROL_DIM = 2
        self.ARM_CONF_DIM = len(arm_joints)

        self.base_collision_links = [3, 4, 5, 6, 7, 9, 10, 14, 15, 19]
        self.wheel_links = [3, 4, 5, 6]
        self.arm_joints = arm_joints

        # Get base params
        r = rospkg.RosPack()
        with open(r.get_path('husky_control') + '/config/control.yaml') as stream:
            parsed_yaml = yaml.safe_load(stream)

        linear_params = parsed_yaml['husky_velocity_controller']['linear']['x']
        angular_params = parsed_yaml['husky_velocity_controller']['angular']['z']

        self.MAX_VELOCITY = torch.tensor([linear_params['max_velocity'], angular_params['max_velocity']],
                                         device=self.device)
        self.MAX_ACCLERATION = torch.diag(
            torch.tensor([linear_params['max_acceleration'], angular_params['max_acceleration']], device=self.device))
        self.time_step = time_step

        MIN_YAW = 0
        MAX_YAW = 2 * np.pi

        self.MIN_LIMITS = torch.tensor([MIN_YAW, -self.MAX_VELOCITY[0], -self.MAX_VELOCITY[1]],
                                       device=self.device).T
        self.MAX_LIMITS = torch.tensor([MAX_YAW, self.MAX_VELOCITY[0], self.MAX_VELOCITY[1]],
                                       device=self.device).T

        # Get arm params
        self.joint_velocity_limits = torch.zeros(self.ARM_CONF_DIM, device=self.device)
        with open(r.get_path('hdt_michigan_moveit') + '/config/joint_limits.yaml') as stream:
            parsed_yaml = yaml.safe_load(stream)
            limits_params = parsed_yaml["joint_limits"]
            for i in range(self.ARM_CONF_DIM):
                joint_name = p.getJointInfo(self.robot, self.arm_joints[i])[1].decode('UTF-8')
                velocity_limit = limits_params[joint_name]["max_velocity"]
                self.joint_velocity_limits[i] = velocity_limit * self.time_step

        # Disable self collision with arms
        # Build dictionary 
        link_name_to_index = {}
        link_name_to_index["base_link"] = -1
        for j in range(p.getNumJoints(self.robot)):
            link_name = p.getJointInfo(self.robot, j)[12].decode('UTF-8')
            link_name_to_index[link_name] = j
        srdf = open(srdf_file).read()
        regex = r'<\s*disable_collisions\s+link1="(\w+)"\s+link2="(\w+)"\s+reason="(\w+)"\s*/>'
        for link1, link2, reason in re.findall(regex, srdf):
            if link1 != "collision_sphere" and link2 != "collision_sphere":
                p.setCollisionFilterPair(self.robot, self.robot, link_name_to_index[link1], link_name_to_index[link2], 0)
        
        # Create joint limits
        self.joint_position_limits = []
        for j in arm_joints:
            joint_info = p.getJointInfo(self.robot, j)
            lower_limit = joint_info[8]
            upper_limit = joint_info[9]
            self.joint_position_limits.append((lower_limit, upper_limit))

        # Disable wheels collision with floor
        for i in self.wheel_links:
            p.setCollisionFilterPair(self.robot, floor, i, -1, 0)

    def get_dynamics_fn(self):
        # Dynamics model for husky
        # x = [xpos, ypos, yaw, v, w]^T
        # u = [u1, u2] where u1, u2 in range [-1, 1] representing -1 as max deaccleration and 1 as max acceleration
        def dynamics_fn(x, u):
            # u has to be in range

            # if np.max(u) > 1 or np.min(u) < -1:
            #     return None
            accel = torch.matmul(u, self.MAX_ACCLERATION) * self.time_step
            new_vels = x[:, 3:5] + accel
            new_vels[:, 0] = torch.clamp(new_vels[:, 0], -self.MAX_VELOCITY[0], self.MAX_VELOCITY[1])
            new_vels[:, 1] = torch.clamp(new_vels[:, 1], -self.MAX_VELOCITY[1], self.MAX_VELOCITY[1])

            delta_distances = new_vels[:, 0:1] * self.time_step
            delta_poses = torch.cat((delta_distances * torch.cos(x[:, 2:3]), delta_distances * torch.sin(x[:, 2:3]),
                                     new_vels[:, 1:2] * self.time_step), 1)
            new_poses = x[:, 0:3] + delta_poses

            xnew = torch.cat((new_poses, new_vels), 1)
            return xnew[0] if x.dim() == 1 else xnew

        return dynamics_fn

    def execute_path(self, start, path, draw_path=False, color=(0, 0, 1)):
        x = start
        if draw_path:
            prev_x = x
            count = 0
        for n in path:
            x, q, _ = n
            self.set_pose(x)
            self.set_joint_configuration(q)
            self.env.step_env()
            if draw_path and not count % 5:
                self.draw_path_line(prev_x, x, color=color)
                prev_x = x
            if draw_path:
                wait_for_duration(self.time_step / 4)
            else:
                wait_for_duration(self.time_step)
            count = count + 1

    def gen_prims(self, num_prims, draw_prims=False):
        prims = []
        if draw_prims:
            dynamics_fn = self.get_dynamics_fn()
        steering_fn = self.get_steering_fn(epsilon=1e-1, ignore_vel=True)

        # Evenly spaced points arround the unit circle
        starts = torch.zeros((num_prims, self.STATE_DIM), device=self.device)
        goals = torch.zeros((num_prims, self.STATE_DIM), device=self.device)
        goals[:, 2] = torch.arange(0, 2 * torch.pi, 2 * np.pi / num_prims, device=self.device)
        goals[:, 0] = 0.5 * torch.cos(goals[:, 2])
        goals[:, 1] = 0.5 * torch.sin(goals[:, 2])

        # Wrap angles
        for yaw in goals[:, 2]:
            if yaw > torch.pi / 2 and yaw < 3 * torch.pi / 2:
                if yaw < torch.pi:
                    yaw += torch.pi
                else:
                    yaw -= torch.pi

        for start, goal in zip(starts, goals):
            prim = steering_fn(start, goal, max_iterations=20)
            if draw_prims:
                x = torch.reshape(start, (1, -1))
                prev_x = x
                for u in prim:
                    x = dynamics_fn(x, u)
                    self.draw_path_line(prev_x, x)
                    prev_x = x
            prims.append(prim)

        return prims

    def get_steering_fn(self, epsilon=1e-1, ignore_vel=False, terminal_state_weight=5):
        dynamics_fn = self.get_dynamics_fn()
        cost_fn = self.get_rs_cost_fn()

        def steering_fn(start, goal, max_iterations=500):
            # give it a terminal cost to encourage actually arriving at the goal
            # running cost becomes a weaker signal closer to the goal so we need an additional cost
            def terminal_cost_fn(current_node, actions):
                return terminal_state_weight * cost_fn(current_node[:, -1], goal), actions

            running_cost_fn = self.get_base_running_cost_fn(goal, cost_fn)
            ctrl = mppi.MPPI(dynamics=dynamics_fn, running_cost=running_cost_fn,
                             nx=self.STATE_DIM, noise_sigma=torch.eye(self.CONTROL_DIM, device=self.device),
                             num_samples=100, horizon=15, device=self.device,
                             u_min=-1 * torch.ones((1, 2), device=self.device),
                             u_max=torch.ones((1, 2), device=self.device))

            x = torch.reshape(start, (1, -1))
            goal = torch.reshape(goal, (1, -1))
            z = []

            for i in range(max_iterations):
                u = torch.reshape(ctrl.command(x), (1, -1))
                z.append(u)
                x = dynamics_fn(x, u)

                if cost_fn(x, goal) < epsilon:
                    break
            return z

        return steering_fn
    
    def get_connect_fn(self):
        def connect_fn(qi, qg, num_steps):
            step = (qg - qi) / num_steps

            # Wrap step size based on velocity constraints
            for i in range(len(step)):
                if abs(step[i]) > self.joint_velocity_limits[i]:
                    step[i] = (step[i] / abs(step[i])) * self.joint_velocity_limits[i]

            # Otherwise connect start and goal linearly (assume holonomic)
            else:
                arm_path = []
                q = qi
                for i in range(num_steps):
                    q = q + step
                    arm_path.append(q)
                return arm_path
        return connect_fn

    # Check collision and step environment
    def get_collision_fn(self):
        def collision_fn(x, q):
            # Set base pose and arm config
            self.set_pose(x)
            self.set_joint_configuration(q)

            p.performCollisionDetection()

            # Check base collision
            for i in self.base_collision_links:
                points = p.getContactPoints(bodyA=self.robot, linkIndexA=i)
                if points:
                    return True
            
            # Check joint limits
            for value, limits in zip(q, self.joint_position_limits):
                if value < limits[0] or value > limits[1]:
                    print("Joint limits exceeded")
                    return True

            # Check self collision
            for i in range(len(self.arm_joints)):
                for j in range(p.getNumJoints(self.robot)):
                    points = p.getContactPoints(bodyA=self.robot, linkIndexA=self.arm_joints[i], bodyB=self.robot, linkIndexB=j)
                    if points:
                        return True

            # Check plant deflections and step environment
            self.env.step_env()
            return self.env.check_deflection()

        return collision_fn

    def get_sample_fn(self):
        def sample_fn():
            base_position = self.env.sample_base_position()
            yaw_vels = (self.MAX_LIMITS - self.MIN_LIMITS) * torch.rand(self.STATE_DIM - 2, device=self.device) + self.MIN_LIMITS
            x = torch.reshape(torch.cat((base_position, yaw_vels)), (1, -1))

            q = torch.zeros(self.ARM_CONF_DIM, device=self.device)
            for i in range(self.ARM_CONF_DIM):
                q[i] = random.uniform(self.joint_position_limits[i][0], self.joint_position_limits[i][1])
            return x, q

        return sample_fn

    def get_base_cost_fn(self, ignore_vel=False, turning_radius=1.0):
        Q = torch.tensor([1, 1, 0.50, 0.25, 0.125], device=self.device)
        if ignore_vel:
            Q = Q[0:3]

        def base_cost_fn(x0, x1):
            diff = x0 - x1
            if ignore_vel:
                diff = diff[:, 0:3]
            diff[:, 2] = angular_diff_batch(x0[:, 2], x1[:, 2])
            cost = diff ** 2 @ Q
            return cost

        return base_cost_fn
    
    def get_rs_cost_fn(self, turning_radius=1.0):
        rs_state = base.ReedsSheppStateSpace(turning_radius)

        def rs_cost_fn(x0, x1):
            if x0.size(0) > x1.size(0):
                x1 = x1.repeat(x0.size()[0], 1)

            rs_costs = torch.zeros(x0.size(0), device=self.device)
            for i in range(x0.size(0)):
                se2_0 = base.SE2StateSpace()
                s_0 = base.State(se2_0)

                s_0().setXY(x0[i, 0].item(), x0[i, 1].item())
                s_0().setYaw(x0[i, 2].item())

                se2_1 = base.SE2StateSpace()
                s_1 = base.State(se2_1)
                s_1().setXY(x1[i, 0].item(), x1[i, 1].item())
                s_1().setYaw(x1[i, 2].item())

                rs_costs[i] = rs_state.distance(s_0(), s_1())
            
            return rs_costs
        
        return rs_cost_fn

    # For use with MPPI controller
    def get_base_running_cost_fn(self, goal, cost_fn):
        R = 0.01

        def base_running_cost_fn(x, u):
            cost = u.norm(dim=1) * R + cost_fn(x, goal.repeat(x.size()[0], 1))
            return cost

        return base_running_cost_fn
    
    def get_arms_cost_fn(self):
        def arms_cost_fn(qi, qg):
            return torch.linalg.norm(qi - qg)
        return arms_cost_fn

    def draw_path_line(self, x1, x2, color=(0, 0, 1), width=1.0):
        self.draw_line((x1[0, 0], x1[0, 1], 0.01), (x2[0, 0], x2[0, 1], 0.01), width, color)

    def draw_line(self, start, end, width, color=(0, 0, 1)):
        line_id = p.addUserDebugLine(start, end, color, width)
        return line_id

    def set_pose(self, x):
        position = (x[0, 0], x[0, 1], self.z_coord)
        rotation = p.getQuaternionFromEuler((0, 0, x[0, 2]))
        p.resetBasePositionAndOrientation(self.robot, position, rotation)
    
    def set_joint_configuration(self, q):
        for j, value in zip(self.arm_joints, q):
            p.resetJointState(self.robot, j, value)
