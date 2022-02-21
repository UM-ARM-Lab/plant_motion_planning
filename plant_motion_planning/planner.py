import pybullet

from plant_motion_planning.env import Environment
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, get_free_motion_gen_with_angle_constraints_multi_world, \
    get_free_motion_gen_multiworld_benchmark
from plant_motion_planning.pybullet_tools.kuka_primitives import Command
from plant_motion_planning.pybullet_tools.utils import save_state


class Planner():

    def __init__(self):

        self.conf1 = None
        self.conf2 = None
        self.command = None

    def move_arm_conf2conf_multi_world(self, init_conf, goal_conf, multi_world_env):
        """
        Method to find a path between init_conf and goal_conf
        """

        # Create a BodyConf object with the initial configuration stored in it
        self.conf1 = BodyConf(multi_world_env.sample_robot, configuration=init_conf)
        # Create a BodyConf object with the final or end configuration stored in it
        self.conf2 = BodyConf(multi_world_env.sample_robot, configuration=goal_conf)

        # Save the initial state of simulation
        start_state_id = save_state()

        # A motion planner function that will be used to find a path
        free_motion_fn = get_free_motion_gen_with_angle_constraints_multi_world(multi_world_env.sample_robot, start_state_id,
                                                                       multi_world_env)

        # Number of attempts at finding a path between conf_i and conf_g
        num_attempts = 200

        path = []
        for attempt in range(num_attempts):

            result = free_motion_fn(self.conf1, self.conf2)

            if result is None or result[0] is None:
                continue
            else:
                path, = result
                self.command = Command(path.body_paths)
                return self.command

        self.command = None
        return self.command

    def move_arm_conf2conf_multi_world_benchmark(self, init_conf, goal_conf, multi_world_env, start_state_id):
        """
        Method to find a path between init_conf and goal_conf
        """

        # Create a BodyConf object with the initial configuration stored in it
        self.conf1 = BodyConf(multi_world_env.sample_robot, configuration=init_conf)
        # Create a BodyConf object with the final or end configuration stored in it
        self.conf2 = BodyConf(multi_world_env.sample_robot, configuration=goal_conf)

        free_motion_fn = get_free_motion_gen_multiworld_benchmark(multi_world_env.sample_robot, multi_world_env.fixed,
                                                                  multi_world_env)

        for num_attempts in range(200):

            pybullet.restoreState(start_state_id)

            path_data = free_motion_fn(self.conf1, self.conf2)

            if (path_data is not None and path_data[0] is not None):
                path = path_data[0]
                self.command = Command(path.body_paths)
                return self.command

        self.command = None
        return self.command

    def find_path(self, conf1, conf2, single_plant_env):
        self.conf1 = conf1
        self.conf2 = conf2
        self.command = self.move_arm_conf2conf(self.robot, self.fixed, self.movable, self.deflection_limit,
                                               conf1, conf2, single_plant_env)

    def execute_path_multi_world(self, multi_world_env):

        if self.command is None:
            print("Error! Command is None!")
        else:
            self.command.execute_multi_world(self.conf1.configuration, multi_world_env)
