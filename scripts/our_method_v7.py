#l!/usr/bin/env python

"""
This program finds a path between two configurations of a robotic arm by deflecting plants within their limits and
without colliding with other objects.

This program only deals with plants with multiple branches and multiple stems. Refer to v4 of our method for a
single stem implementation.
"""

from __future__ import print_function

import numpy as np
import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.env import SinglePlantEnv
from plant_motion_planning.planner import Planner
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, Command, get_free_motion_gen_with_angle_constraints_v6
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, update_state, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT, save_state, set_joint_positions, get_movable_joints
import pybullet as p
from plant_motion_planning.utils import load_plant_from_urdf

import os
import argparse


# Argument parser
parser = argparse.ArgumentParser(description='')
parser.add_argument("--video_filename", type=str, help="File name of video file to record planning with full path",
                    default=None)
parser.add_argument("--env", type=str, help="Name of environment to be used", default=None)
parser.add_argument("--deflection_limit", type=float, help="Maximum amount of deflection to be permitted", default=None)
parser.add_argument("--plant_filename", type=str, help="Path to locations of plant model files",
                    default=None)
args = parser.parse_args()


def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g):
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

    # Save the initial state of simulation
    start_state_id = save_state()

    # A motion planner function that will be used to find a path
    free_motion_fn = get_free_motion_gen_with_angle_constraints_v6(robot, start_state_id, fixed= fixed,
                                                                   movable = movable,
                                                                   deflection_limit = deflection_limit)

    # Number of attempts at finding a path between conf_i and conf_g
    num_attempts = 200

    path = []
    for attempt in range(num_attempts):

        result = free_motion_fn(conf_i, conf_g)

        if result is None or result[0] is None:
            continue
        else:
            path, = result
            return Command(path.body_paths)

    return None


# Initial configuration of the Arm
init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Final configuration of the Arm
goal_conf = (-1.3871757013351371, 1.6063773991870438, 2.152853076950719, -1.0638334445672613, -0.1398096715085235,
             1.9391079682303638, -1.051507203470595)


def main(display='execute'): # control | execute | step
    """
    Main function of this program.

    :param:
        display: Type of execution of the program. eg. full path execution, step by step execution etc.
    :return: -
    """

    # GUI connection client object
    # connect(use_gui=True,width=1000, height=700)
    # cli = connect(use_gui=True,width=1920, height=1080)
    cli = connect(use_gui=True,width=1000, height=700)
    disable_real_time()

    # Set camera pose to desired position and orientation
    set_camera_pose((0.0, -1.06, 1.5),(0,0,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    # Draw X, Y, Z axes
    draw_global_system()
    with HideOutput():
        # Load robot model given macro to urdf file and fix its base
        robot = load_model(DRAKE_IIWA_URDF_EDIT, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # Load floor model from urdf file and fix its base
        floor = load_model(os.path.join(cfg.ROOT_DIR, 'models/short_floor.urdf'), fixed_base=True)

    # Load a block that we would like to use as our object of interest
    block = load_model(BLOCK_URDF, fixed_base=True)
    # Set location and orientation of block in the simulation world
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))


    # Generate a new plant whose number of branches, number of stems, natural deflections etc. are randomly sampled
    num_branches_per_stem = 1
    total_num_vert_stems = 2
    total_num_extensions = 1
    # base_pos_xy = [np.random.uniform(low=0, high=0.35),np.random.uniform(low=-0.5, high=0.0)]
    # # base_pos_xy = [0.4, 0.4]
    # plant_id, plant_rep = generate_random_plant(num_branches_per_stem, total_num_vert_stems, total_num_extensions,
    #                                             base_pos_xy, physicsClientId=cli)


    joints = get_movable_joints(robot)
    set_joint_positions(robot, joints, goal_conf)

    # initialize deflection limit from arguments input
    deflection_limit = 0.30

    # Base position in (X, Y)
    base_pos_xy = [np.random.uniform(low=0, high=0.35), np.random.uniform(low=-0.5, high=0.0)]
    print("Base position: ", base_pos_xy)

    fixed = [floor, block]

    # Generate a new random plant
    # single_plant_env = SinglePlantEnv(robot, fixed, deflection_limit, num_branches_per_stem, total_num_vert_stems,
    #                                   total_num_extensions, base_pos_xy)

    # Load plant from urdf file
    single_plant_env = SinglePlantEnv(robot, fixed, deflection_limit, base_pos_xy=base_pos_xy,
                                      loadPath=args.plant_filename)

    # TODO: Planning...
    # Create a BodyConf object with the initial configuration stored in it
    conf_i = BodyConf(robot, configuration=init_conf)
    # Create a BodyConf object with the final or end configuration stored in it
    conf_g = BodyConf(robot, configuration=goal_conf)

    planner = Planner(robot)

    saved_world = p.saveState()

    planner.move_arm_conf2conf(conf_i, conf_g, single_plant_env)
    print("Planning completed!")

    p.restoreState(saved_world)
    update_state()

    # Save a video of the execution if required
    if(args.video_filename != None):
        log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=args.video_filename)

    # TODO: Execution...
    planner.execute_path(single_plant_env)
    print("execution completed!")

    # shutting down logger
    if(args.video_filename != None):
        p.stopStateLogging(log_id)

    print('Quitting simulation')
    disconnect()

if __name__ == '__main__':
    main(display = "angle_step")
