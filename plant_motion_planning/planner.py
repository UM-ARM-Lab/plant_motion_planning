from plant_motion_planning.env import Environment
from plant_motion_planning.pybullet_tools.kuka_primitives import get_free_motion_gen_with_angle_constraints_v6, \
    get_free_motion_gen_with_angle_constraints_v7
from plant_motion_planning.pybullet_tools.kuka_primitives import Command
from plant_motion_planning.pybullet_tools.utils import save_state


class Planner():

    # def __init__(self, env: Environment, robot):
    def __init__(self, robot):

        # self.env = environment()
        self.robot = robot
        # self.fixed = fixed
        # self.movable = movable
        # self.deflection_limit = deflection_limit
        # self.plant = singlePlantEnv()

        self.conf1 = None
        self.conf2 = None
        self.command = None


    def move_arm_conf2conf(self, conf_i, conf_g, single_plant_env):
        """
        Method to find a path between conf_i and conf_g

        :param:
            robot: Body ID of robot returned by pybullet
            fixed: Body IDs of entities that are fixed during simulation. These are the objects that collision will be
            checked against.
            movable: Characterization objects of the plants
            deflection_limit: The maximum amount of deflection each link of the plant can undergo
            conf_i: BodyConf object denoting the initial configuration
            conf_g: BodyConf object denotion the final or goal configuration

        :return:
            A Command object that contains the path(s) from conf_i to conf_g if a path exists. Else, if no path exists, it
            return None.
        """

        self.conf1 = conf_i
        self.conf2 = conf_g

        # Save the initial state of simulation
        start_state_id = save_state()

        # A motion planner function that will be used to find a path
        # free_motion_fn = get_free_motion_gen_with_angle_constraints_v6(robot, start_state_id, fixed= fixed,
        #                                                                movable = movable,
        #                                                                deflection_limit = deflection_limit,
        #                                                                single_plant_env = single_plant_env)
        free_motion_fn = get_free_motion_gen_with_angle_constraints_v7(self.robot, start_state_id, single_plant_env)

        # Number of attempts at finding a path between conf_i and conf_g
        num_attempts = 200

        path = []
        for attempt in range(num_attempts):

            result = free_motion_fn(conf_i, conf_g)

            if result is None or result[0] is None:
                continue
            else:
                path, = result
                self.command = Command(path.body_paths)
                return self.command

        self.command = None
        return self.command

    def find_path(self, conf1, conf2, single_plant_env):
        self.conf1 = conf1
        self.conf2 = conf2
        self.command = self.move_arm_conf2conf(self.robot, self.fixed, self.movable, self.deflection_limit,
                                               conf1, conf2, single_plant_env)

    def execute_path(self, single_plant_env):

        if self.command is None:
            print("Error! Command is None!")
        else:
            self.command.execute_with_controls_v3(self.robot, self.conf1.configuration, single_plant_env)