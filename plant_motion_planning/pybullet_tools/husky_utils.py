import numpy as np
import pybullet as p
import rospkg
import torch
import yaml
from arm_pytorch_utilities.math_utils import angular_diff_batch
from pytorch_mppi import mppi

from plant_motion_planning.pybullet_tools.utils import step_simulation, wait_for_duration

r = rospkg.RosPack()
with open(r.get_path('husky_control') + '/config/control.yaml') as stream:
    parsed_yaml = yaml.safe_load(stream)

linear_params = parsed_yaml['husky_velocity_controller']['linear']['x']
angular_params = parsed_yaml['husky_velocity_controller']['angular']['z']

MAX_VELOCITY = torch.tensor([linear_params['max_velocity'], angular_params['max_velocity']])
MAX_ACCLERATION = torch.diag(torch.tensor([linear_params['max_acceleration'], angular_params['max_acceleration']]))
TIME_STEP = 1./240. * 50

STATE_DIM = 5
CONTROL_DIM = 2

MIN_X = -5
MIN_Y = -5
MIN_YAW = 0

MAX_X = 5
MAX_Y = 5
MAX_YAW = 2 * np.pi

MIN_LIMITS = torch.tensor([MIN_X, MIN_Y, MIN_YAW, -MAX_VELOCITY[0], -MAX_VELOCITY[1]]).T
MAX_LIMITS = torch.tensor([MAX_X, MAX_Y, MAX_YAW, MAX_VELOCITY[0], MAX_VELOCITY[1]]).T

def get_dynamics_fn():
    # Dynamics model for husky
    # x = [xpos, ypos, yaw, v, w]^T
    # u = [u1, u2] where u1, u2 [-1, 1] representing -1 as max deaccleration and 1 as max acceleration
    def dynamics_fn(x, u):
        # u has to be in range

        # if np.max(u) > 1 or np.min(u) < -1:
        #     return None
        
        accel = torch.matmul(u, MAX_ACCLERATION) * TIME_STEP
        new_vels = x[:, 3:5] + accel
        new_vels[:, 0] = torch.clamp(new_vels[:, 0], -MAX_VELOCITY[0], MAX_VELOCITY[1])
        new_vels[:, 1] = torch.clamp(new_vels[:, 1], -MAX_VELOCITY[1], MAX_VELOCITY[1])

        delta_distances = new_vels[:, 0:1] * TIME_STEP
        delta_poses = torch.cat((delta_distances * torch.cos(x[:, 2:3]), delta_distances * torch.sin(x[:, 2:3]), new_vels[:, 1:2] * TIME_STEP), 1)
        new_poses = x[:, 0:3] + delta_poses

        xnew = torch.cat((new_poses, new_vels), 1)
        return xnew[0] if x.dim() == 1 else xnew
    return dynamics_fn

def execute_path(robot, start, path, dynamics_fn, draw_path = False):
    x = start
    if draw_path:
        prev_x = x
        count = 0
    for z in path:
        for u in z:
            x = dynamics_fn(x, u)
            set_pose(robot, x)
            if draw_path and not count % 5:
                draw_path_line(prev_x, x)
                prev_x = x

            wait_for_duration(TIME_STEP)

def gen_prims(num_prims, dynamics_fn, device, draw_prims=False):
    prims = []

    # Evenly spaced points arround the unit circle
    starts = torch.zeros((num_prims, STATE_DIM), device=device)
    goals = torch.zeros((num_prims, STATE_DIM), device=device)
    goals[:, 2] = torch.arange(0, 2*torch.pi, 2*np.pi / num_prims, device=device)
    goals[:, 0] = 1.5*torch.cos(goals[:, 2])
    goals[:, 1] = 1.5*torch.sin(goals[:, 2])
    
    # Wrap angles
    for yaw in goals[:, 2]:
        if yaw > torch.pi / 2 and yaw < 3 * torch.pi / 2:
            if yaw < torch.pi:
                yaw += torch.pi
            else:
                yaw -= torch.pi

    for start, goal in zip(starts, goals):
        prim = local_planner(start, goal, dynamics_fn, device, ignore_vel=True)
        prims.append(prim)

    return prims

def local_planner(start, goal, dynamics_fn, device, ignore_vel=False, epsilon=2e-1, max_iterations=1000):
    cost_fn = get_cost_fn(ignore_vel=ignore_vel)
    running_cost_fn = get_running_cost_fn(goal, ignore_vel=ignore_vel)
    ctrl = mppi.MPPI(dynamics=dynamics_fn, running_cost=running_cost_fn, nx=STATE_DIM, noise_sigma=torch.eye(CONTROL_DIM, device=device), \
                        num_samples=1000, device=device, u_min=-1*torch.ones((1, 2), device=device), u_max=torch.ones((1, 2), device=device))

    x = torch.reshape(start, (1, -1))
    goal = torch.reshape(goal, (1, -1))
    controls = torch.empty((0, 2) ,device=device)

    for i in range(max_iterations):
        u = torch.reshape(ctrl.command(x), (1, -1))
        controls = torch.cat((controls, u), dim=0)
        x = dynamics_fn(x, u)

        if cost_fn(x, goal) < epsilon:
            break
    return controls

def get_collision_fn():
    # TODO write actual collision 
    def collision_fn(x):
        return False
    return collision_fn

def get_sample_fn():
    def sample_fn():
        sample = (MIN_LIMITS - MAX_LIMITS) * torch.rand(STATE_DIM) + MIN_LIMITS
        return torch.reshape(sample, (1, -1))
    return sample_fn

def get_cost_fn(ignore_vel=False):
    Q = torch.tensor([1, 1, 0.3, 3, 0.1])
    if ignore_vel:
        Q = Q[0:3]

    def cost_fn(x0, x1):
        diff = x0 - x1
        if ignore_vel:
            diff = diff[:, 0:3]
        diff[:, 2] = angular_diff_batch(x0[:, 2], x1[:, 2])
        cost = diff ** 2 @ Q
        return cost
    return cost_fn

# For use with MPPI controller
def get_running_cost_fn(goal, ignore_vel=False):
    R = 0.01
    cost_fn = get_cost_fn(ignore_vel=ignore_vel)

    def running_cost_fn(x, u):
        cost = u.norm(dim=1) * R + cost_fn(x, goal.repeat(x.size()[0], 1))
        return cost
    return running_cost_fn

def draw_path_line(x1, x2, color=(0, 0, 1), width = 1.0):
    draw_line((x1[0, 0], x1[0, 1], 0.01), (x2[0, 0], x2[0, 1], 0.01), width, color)

def draw_line(start, end, width, color=(0, 0, 1)):
    line_id = p.addUserDebugLine(start, end, color, width)
    return line_id

def set_pose(robot, x):
    position = (x[0, 0], x[0, 1], 0.31769884443141244)
    rotation = p.getQuaternionFromEuler((0, 0, x[0, 2]))
    p.resetBasePositionAndOrientation(robot, position, rotation)