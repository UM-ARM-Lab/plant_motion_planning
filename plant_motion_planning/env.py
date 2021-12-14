import abc
import os

import pybullet as p
import plant_motion_planning.pybullet_tools.utils as pyb_tools_utils
from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import get_free_motion_gen_with_angle_constraints_v6, Command
from plant_motion_planning.utils import generate_random_plant, step_sim_v2, load_plant_from_urdf


# global cli

# fixed = []



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

    def __init__(self, deflection_limit, num_branches_per_stem=None, total_num_vert_stems=None,
                 total_num_extensions=None, plant_pos_xy=(0, 0), base_offset_xy=(0, 0), loadPath=None, physicsClientId=None):

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
        pose_robot = pyb_tools_utils.Pose(pyb_tools_utils.Point(x=base_offset_xy[0], y=base_offset_xy[1], z=0))
        self.robot = pyb_tools_utils.load_model(pyb_tools_utils.DRAKE_IIWA_URDF_EDIT, pose=pose_robot, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF

        if(loadPath is None):
            self.plant_id, self.plant_rep, self.joint_list, self.base_id = generate_random_plant(num_branches_per_stem,
                                                                    total_num_vert_stems, total_num_extensions,
                                                                                                 (plant_pos_xy[0] + base_offset_xy[0],
                                                                                                  plant_pos_xy[1] + base_offset_xy[1]),
                                                                                                 physicsClientId=physicsClientId)
        else:
            self.plant_id, self.plant_rep, self.joint_list, self.base_id = load_plant_from_urdf(os.path.join(loadPath, "plant.urdf"),
                                                                                                os.path.join(loadPath, "plant_params.pkl"), base_offset_xy)

        self.fixed = [floor, block]
        self.movable = [self.plant_rep]

        self.deflection_limit = deflection_limit

        self.joints = pyb_tools_utils.get_movable_joints(self.robot)

        # self.joint_list = []
        # self.base_id = []

        self.collision_fn = pyb_tools_utils.get_collision_fn_with_angle_constraints_v4(self.robot, self.fixed, self.joints,
                                                                                       self.movable,
                                                                                       self.deflection_limit, [],
                                                                                       True,
                                                                                       set(),
                                                                                       custom_limits={},
                                                                                       max_distance=pyb_tools_utils.MAX_DISTANCE,
                                                                                       use_aabb=False, cache=True)

        self.collision_fn_back = pyb_tools_utils.get_collision_fn_with_angle_constraints_v4(self.robot, self.fixed, self.joints,
                                                                                       [],
                                                                                       self.deflection_limit, [],
                                                                                       True,
                                                                                       set(),
                                                                                       custom_limits={},
                                                                                       max_distance=pyb_tools_utils.MAX_DISTANCE,
                                                                                       use_aabb=False, cache=True)

        super(SinglePlantEnv, self).__init__()

    def _restore_plant_joints(self):
        kp = 3000

        for i, joint_idx in enumerate(range(len(self.joint_list) - 1, 0, -2)):
            plant_rot_joint_displacement_y, _, plant_hinge_x_reac, _ = p.getJointState(self.base_id, joint_idx)
            plant_rot_joint_displacement_x, _, plant_hinge_y_reac, _ = p.getJointState(self.base_id, joint_idx - 1)

            p.applyExternalTorque(self.base_id, linkIndex=joint_idx,
                                  torqueObj=[-kp * plant_rot_joint_displacement_x,
                                             -kp * plant_rot_joint_displacement_y, 0],
                                  flags=p.LINK_FRAME)

    def step(self, action, set_joint_pos=False):

        if(set_joint_pos):
            pyb_tools_utils.set_joint_positions(self.robot, self.joints, action)
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, action, positionGains=7 * [0.01])


        for i in range(11):

            # Restore plant joints after deflection
            self._restore_plant_joints()

            # stepping through simulation
            for t in range(200):
                pyb_tools_utils.step_simulation()

        return None


    def step_after_restoring(self, action, set_joint_pos=True):

        if(set_joint_pos):
            pyb_tools_utils.set_joint_positions(self.robot, self.joints, action)
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, action, positionGains=7 * [0.01])


        # for i in range(11):
        #
        #     # Restore plant joints after deflection
        #     self._restore_plant_joints()
        #
        #     # stepping through simulation
        #     for t in range(200):
        #         pyb_tools_utils.step_simulation()

        return None

    def state(self):
        deflections = self.plant_rep.deflections
        self.full_env_current_state = {'robot': p.getJointState(self.robot), 'plants': deflections}
        return deflections

    def constraint_violation_checker_forward(self, q):
        return self.collision_fn(q)
    def constraint_violation_checker_backward(self, q):
        return self.collision_fn_back(q)


class MultiPlantWorld(Environment):
    def __init__(self, xs, ys, *args, **kwargs):
        self.envs = {}
        for x in xs:
            for y in ys:
                xy = (x, y)
                self.envs[xy] = SinglePlantEnv(*args, base_offset_xy=xy, **kwargs)
        super(MultiPlantWorld, self).__init__()

        self.sample_robot = self.envs[(xs[0], ys[0])].robot
        self.joints = self.envs[(xs[0], ys[0])].joints

    # def step(self, action, set_joint_pos=False):
    #     ret = {}
    #     # step all the environments with the same action
    #     for xy, env in self.envs.items():
    #         ret[xy] = env.step(action, set_joint_pos=set_joint_pos)
    #
    #     return ret

    def step(self, action, set_joint_pos=False):
        for xy, env in self.envs.items():
            env.step_after_restoring(action, set_joint_pos=set_joint_pos)

        for t in range(11):
            for xy, env in self.envs.items():
                env._restore_plant_joints()


            for t in range(200):
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
