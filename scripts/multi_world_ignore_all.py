#l!/usr/bin/env python

"""
This program finds a path between initial and goal configurations such that plants are deflected without caring about their deflection limits.
"""

from __future__ import print_function

import random

import numpy as np
import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.env import SinglePlantEnv, MultiPlantWorld
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
parser.add_argument("--deflection_limit", type=float, help="Maximum amount of deflection to be permitted", default=None)
parser.add_argument("--plant_filename", type=str, help="Path to locations of plant model files",
                    default=None)
args = parser.parse_args()


# Initial configuration of the Arm
init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Final configuration of the Arm
goal_conf = (-1.3871757013351371, 1.6063773991870438, 2.152853076950719, -1.0638334445672613, -0.1398096715085235,
             1.9391079682303638, -1.051507203470595)


def main():
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
    set_camera_pose((2.5, -1.06, 3.5),(2.5,2.5,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    # Draw X, Y, Z axes
    draw_global_system()

    # Example environments... (Uncomment one of them and comment others to use)
    # Env 1
    # Initializing random generator
    np.random.seed(1)
    random.seed(1)
    # Base position in X and Y
    plant_pos_xy_limits = ((0, 0.35), (-0.5, 0.0))

    # Env2
    # np.random.seed(19)
    # random.seed(19)
    # plant_pos_xy_limits = ((0.2, 0.35), (-0.5, 0.0))

    # Env 3
    # np.random.seed(2)
    # random.seed(2)
    # plant_pos_xy_limits = ((0, 0.35), (-0.5, 0.0))

    # Generate a new plant whose number of branches, number of stems, natural deflections etc. are randomly sampled
    num_branches_per_stem = 1
    total_num_vert_stems = 2
    total_num_extensions = 1

    # initialize deflection limit from arguments input
    deflection_limit = 0.30

    # Base offset in X and Y
    base_offset_xs = (0, 5)
    base_offset_ys = (0, 5)

    # Generate random plant for each world
    multi_world_env = MultiPlantWorld(base_offset_xs, base_offset_ys, deflection_limit, num_branches_per_stem, total_num_vert_stems,
                                      total_num_extensions, plant_pos_xy_limits, physicsClientId=cli)

    # Restarting random generator
    np.random.seed()
    random.seed()

    planner = Planner()

    saved_world = p.saveState()

    planner.move_arm_conf2conf_multi_world_benchmark(init_conf, goal_conf, multi_world_env, saved_world)
    print("Planning completed!")

    p.restoreState(saved_world)
    update_state()

    # Save a video of the execution if required
    if(args.video_filename != None):
        log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=args.video_filename)

    # TODO: Execution...
    planner.execute_path_multi_world(multi_world_env)
    print("execution completed!")

    # shutting down logger
    if(args.video_filename != None):
        p.stopStateLogging(log_id)

    print('Quitting simulation')
    disconnect()

if __name__ == '__main__':
    main()
