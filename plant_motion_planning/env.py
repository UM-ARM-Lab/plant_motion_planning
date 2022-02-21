import abc
import os

import numpy as np
import pybullet as p
import plant_motion_planning.pybullet_tools.utils as pyb_tools_utils
from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import Command
from plant_motion_planning.pybullet_tools.val_utils import get_arm_joints
from plant_motion_planning.utils import generate_random_plant, step_sim_v2, load_plant_from_urdf



class Environment():

    def __init__(self):

        pass

    @abc.abstractmethod
    def step(self, action):
        """Take a simulation step in this environment"""

    @abc.abstractmethod
    def state(self):
        """Retrieve the current state in a format suitable for a planner"""


class SinglePlantEnv(Environment):

    """
    Single plant environment
    """

    def __init__(self, deflection_limit, num_branches_per_stem=None, total_num_vert_stems=None,
                 total_num_extensions=None, plant_pos_xy_limits=((0, 1), (0, 1)), base_offset_xy=(0, 0), loadPath=None, physicsClientId=None, avoid_all=False):

        pose_floor = pyb_tools_utils.Pose(pyb_tools_utils.Point(x=base_offset_xy[0], y=base_offset_xy[1], z=0))
        floor = pyb_tools_utils.load_model(os.path.join(cfg.ROOT_DIR, 'models/short_floor.urdf'), pose=pose_floor,
                                           fixed_base=True)

        # Load a block that we would like to use as our object of interest
        block = pyb_tools_utils.load_model(pyb_tools_utils.BLOCK_URDF, fixed_base=True)
        # Set location and orientation of block in the simulation world
        pyb_tools_utils.set_pose(block, pyb_tools_utils.Pose(pyb_tools_utils.Point(x=0.4 + base_offset_xy[0],
                                                                                   y=-0.4 + base_offset_xy[1], z=0.45),
                                                             pyb_tools_utils.Euler(yaw=1.57)))

        # Load robot model given macro to urdf file and fix its base
        pose_robot = pyb_tools_utils.Pose(pyb_tools_utils.Point(x=base_offset_xy[0] + 0.02, y=base_offset_xy[1] - 0.31, z=0.132955), pyb_tools_utils.Euler(yaw=np.pi/2))
        self.robot = pyb_tools_utils.load_model(pyb_tools_utils.HDT_MICHIGAN_URDF, pose=pose_robot, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF

        # Let robot fall to ground before loading in plants
        for t in range(0, 50):
            pyb_tools_utils.step_simulation()
        
        if(loadPath is None):

            # num_branches_per_stem = np.random.randint(1, 4)
            # total_num_vert_stems = np.random.randint(2, 4)
            # total_num_extensions = np.random.randint(1, 3)

            plant_pos_xy = [np.random.uniform(low=plant_pos_xy_limits[0][0], high=plant_pos_xy_limits[0][1]),
                            np.random.uniform(low=plant_pos_xy_limits[1][0], high=plant_pos_xy_limits[1][1])]

            self.plant_id, self.plant_rep, self.joint_list, self.base_id = generate_random_plant(num_branches_per_stem,
                                                                    total_num_vert_stems, total_num_extensions,
                                                                                                 (plant_pos_xy[0] + base_offset_xy[0],
                                                                                                  plant_pos_xy[1] + base_offset_xy[1]),
                                                                                                 physicsClientId=physicsClientId)
        else:
            self.plant_id, self.plant_rep, self.joint_list, self.base_id = load_plant_from_urdf(os.path.join(loadPath, "plant.urdf"),
                                                                                                os.path.join(loadPath, "plant_params.pkl"), base_offset_xy)

        self.fixed = [floor, block]
        if(avoid_all):
            self.fixed = self.fixed + [self.plant_id]

        self.movable = [self.plant_rep]

        self.deflection_limit = deflection_limit

        #self.joints = pyb_tools_utils.get_movable_joints(self.robot)
        self.joints = get_arm_joints(self.robot, is_left=True, include_torso=False)

        # Get disabled collisions
        _link_name_to_index = {pyb_tools_utils.get_body_info(self.robot)[0].decode('UTF-8'):-1,}
        for _id in range(pyb_tools_utils.get_num_joints(self.robot)):
            _name = pyb_tools_utils.get_joint_info(self.robot,_id).linkName.decode('UTF-8')
            _link_name_to_index[_name] = _id
        disabled_collisions = pyb_tools_utils.load_srdf_collisions(pyb_tools_utils.HDT_MICHIGAN_SRDF, _link_name_to_index)

        # self.joint_list = []
        # self.base_id = []

        self.collision_fn = pyb_tools_utils.get_collision_fn_with_angle_constraints_v4(self.robot, self.fixed, self.joints,
                                                                                       self.movable,
                                                                                       self.deflection_limit, [],
                                                                                       True,
                                                                                       disabled_collisions,
                                                                                       custom_limits={},
                                                                                       max_distance=pyb_tools_utils.MAX_DISTANCE,
                                                                                       use_aabb=False, cache=True)

        self.collision_fn_back = pyb_tools_utils.get_collision_fn_with_angle_constraints_v4(self.robot, self.fixed, self.joints,
                                                                                       [],
                                                                                       self.deflection_limit, [],
                                                                                       True,
                                                                                       disabled_collisions,
                                                                                       custom_limits={},
                                                                                       max_distance=pyb_tools_utils.MAX_DISTANCE,
                                                                                       use_aabb=False, cache=True)

        self.collision_fn_benchmark = pyb_tools_utils.get_collision_fn_with_controls_benchmark_multiworld(self.robot,
                                                                                                          self.joints,
                                                                                                          self.fixed, [],
                                                                                                          True,
                                                                                                          set(),
                                                                                                          custom_limits={}, max_distance=pyb_tools_utils.MAX_DISTANCE,
                                                                                                          use_aabb=False, cache=True)

        self.prev_plant_rot_joint_displacement_y = (len(self.joint_list) // 2) * [0]
        self.prev_plant_rot_joint_displacement_x = (len(self.joint_list) // 2) * [0]
        
        super(SinglePlantEnv, self).__init__()

    def _restore_plant_joints(self):
        kp = 500
        kd = 50

        for i, joint_idx in enumerate(range(len(self.joint_list) - 1, 0, -2)):
            plant_rot_joint_displacement_y, _, plant_hinge_x_reac, _ = p.getJointState(self.base_id, joint_idx)
            plant_rot_joint_displacement_x, _, plant_hinge_y_reac, _ = p.getJointState(self.base_id, joint_idx - 1)

            diffy = plant_rot_joint_displacement_y - self.prev_plant_rot_joint_displacement_y[i]
            diffx = plant_rot_joint_displacement_x - self.prev_plant_rot_joint_displacement_x[i]

            self.prev_plant_rot_joint_displacement_y[i], self.prev_plant_rot_joint_displacement_x[i] = plant_rot_joint_displacement_y, \
                                                                                             plant_rot_joint_displacement_x

            p.applyExternalTorque(self.base_id, linkIndex=joint_idx,
                                  torqueObj=[-kp * plant_rot_joint_displacement_x + kd * diffx,
                                             -kp * plant_rot_joint_displacement_y + kd * diffy, 0],
                                  flags=p.LINK_FRAME)

    def step(self, action, set_joint_pos=False):

        """
        Customized step simulation function
        """

        if(set_joint_pos):
            pyb_tools_utils.set_joint_positions(self.robot, self.joints, action)
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, action, positionGains=len(self.joints) * [0.01])

        # stepping through simulation
        for t in range(50):
            # Restore plant joints after deflection
            self._restore_plant_joints()

            pyb_tools_utils.step_simulation()
            
        return None


    def step_after_restoring(self, action, set_joint_pos=True):

        if(set_joint_pos):
            pyb_tools_utils.set_joint_positions(self.robot, self.joints, action)
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, action, positionGains=len(self.joints) * [0.01])

        return None

    def state(self):
        deflections = self.plant_rep.deflections
        self.full_env_current_state = {'robot': p.getJointState(self.robot), 'plants': deflections}
        return deflections

    def constraint_violation_checker_forward(self, q):
        return self.collision_fn(q)
    def constraint_violation_checker_backward(self, q):
        return self.collision_fn_back(q)

    def constraint_violation_checker_benchmark(self, q):
        return self.collision_fn_benchmark(q)

class MultiPlantWorld(Environment):

    """
    Multi world environment
    """

    def __init__(self, xs, ys, *args, **kwargs):
        self.envs = {}
        for x in xs:
            for y in ys:
                xy = (x, y)
                self.envs[xy] = SinglePlantEnv(*args, base_offset_xy=xy, **kwargs)
        super(MultiPlantWorld, self).__init__()

        self.sample_robot = self.envs[(xs[0], ys[0])].robot
        self.joints = self.envs[(xs[0], ys[0])].joints
        self.fixed = self.envs[(xs[0], ys[0])].fixed
        self.deflection_limit = self.envs[(xs[0], ys[0])].deflection_limit

    def step_no_action(self):
        for t in range(50):
            for xy, env in self.envs.items():
                env._restore_plant_joints()
            pyb_tools_utils.step_simulation()

    def step(self, action, set_joint_pos=False):
        for xy, env in self.envs.items():
            env.step_after_restoring(action, set_joint_pos=set_joint_pos)

        for t in range(50):
            for xy, env in self.envs.items():
                env._restore_plant_joints()
            pyb_tools_utils.step_simulation()


    def step_after_restoring(self, action):

        for xy, env in self.envs.items():
            env.step_after_restoring(action)

        self.step(action)


    def net_constraint_violation_checker_forward(self, q):

        for xy, env in self.envs.items():

            if(env.collision_fn(q)):

                return True

        return False


    def net_constraint_violation_checker_backward(self, q):
        for xy, env in self.envs.items():

            if (env.collision_fn_back(q)):
                return True

        return False

    def net_constraint_violation_checker_benchmark(self, q):

        for xy, env in self.envs.items():
            if (env.collision_fn_benchmark(q)):
                return True

        return False
