import abc
import os

import pybullet as p
import plant_motion_planning.pybullet_tools.utils as pyb_tools_utils
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

    def __init__(self, robot, fixed, deflection_limit, num_branches_per_stem=None, total_num_vert_stems=None,
                 total_num_extensions=None, base_pos_xy=(0, 0), loadPath=None):

        self.robot = robot

        if(loadPath is None):
            self.plant_id, self.plant_rep, self.joint_list, self.base_id = generate_random_plant(num_branches_per_stem,
                                                                    total_num_vert_stems, total_num_extensions, base_pos_xy)
        else:
            self.plant_id, self.plant_rep, self.joint_list, self.base_id = load_plant_from_urdf(os.path.join(loadPath, "plant.urdf"),
                                                                                                os.path.join(loadPath, "plant_params.pkl"))

        self.fixed = fixed
        self.movable = [self.plant_rep]

        self.deflection_limit = deflection_limit

        self.joints = pyb_tools_utils.get_movable_joints(self.robot)

        # self.joint_list = []
        # self.base_id = []

        self.full_env_current_state = []

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

    def step(self, action):

        # TODO: Action vs State
        # Move arm to given state.
        p.setJointMotorControlArray(self.robot, self.joints, p.POSITION_CONTROL, action, positionGains=7 * [0.01])


        for i in range(11):

            # Restore plant joints after deflection
            self._restore_plant_joints()

            # stepping through simulation
            for t in range(200):
                pyb_tools_utils.step_simulation()


    def state(self):
        deflections = self.plant_rep.deflections
        self.full_env_current_state = {'robot': p.getJointState(self.robot), 'plants': deflections}
        return deflections




class MultiPlantWorld(Environment):
    def __init__(self, xs, ys, *args, **kwargs):
        self.envs = {}
        for x in xs:
            for y in ys:
                xy = (x,y)
                self.envs[xy] = SinglePlantEnv(*args, base_pos_xy=xy, **kwargs)
        super(MultiPlantWorld, self).__init__()

    def step(self, action):
        ret = {}
        # step all the environments with the same action
        for xy, env in self.envs.items():
            ret[xy] = env.step(action)

        return ret
