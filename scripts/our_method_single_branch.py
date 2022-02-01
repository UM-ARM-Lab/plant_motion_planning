#l!/usr/bin/env python

"""
This program finds a path between two configurations of a robotic arm by deflecting plants within their limits and
without colliding with other objects.

This program only deals with plants with single stems. Refer to our_method_multi_branch.py for a multi-branch
implementation.
"""

from __future__ import print_function
from fileinput import filename
from mimetypes import init
from ntpath import join

import os
import cProfile
import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, Command, get_free_motion_gen_with_angle_constraints_v4
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, get_link_name, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, update_state, \
    disable_real_time, HideOutput, HDT_MICHIGAN_URDF, save_state, step_simulation, set_joint_positions, get_joint_limits, wait_for_duration
from plant_motion_planning.pybullet_tools.val_utils import get_arm_joints
import pybullet as p
from plant_motion_planning.utils import envs, generate_tall_plants
import argparse


# Argument parser
parser = argparse.ArgumentParser(description='')
parser.add_argument("--video_filename", type=str, help="File name of video file to record planning with full path",
                    default=None)
parser.add_argument("--env", type=str, help="Name of environment to be used", default=None)
parser.add_argument("--deflection_limit", type=float, help="Maximum amount of deflection to be permitted", default=None)
args = parser.parse_args()


if(args.env == None or args.deflection_limit == None):
    print("Error!! All data must be provided")
    exit()


def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g, teleport=False):
    """
    Method to find a path between conf_i and conf_g

    :param:
        robot: Body ID of robot returned by pybullet
        fixed: Body IDs of entities that are fixed during simulation. These are the objects that collision will be
        checked against.
        conf_i: BodyConf object denoting the initial configuration
        conf_g: BodyConf object denotion the final or goal configuration

    :return:
        A Command object that contains the path(s) from conf_i to conf_g if a path exists. Else, if no path exists, it
        return None.
    """

    # Save the initial state of simulation
    start_state_id = save_state()

    # A motion planner function that will be used to find a path
    free_motion_fn = get_free_motion_gen_with_angle_constraints_v4(robot, start_state_id, fixed= fixed, movable = movable,
                                                                   deflection_limit = deflection_limit)
    # free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)

    # Number of attempts at finding a path between conf_i and conf_g
    num_attempts = 1

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
init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# Final configuration of the Arm
goal_conf = (-2.3, 2.82, 1.48, -0.27, -5.18, -0.55, 4.23, -0.08, 0.70)


def main():
    """
    Main function of this program.
    """

    # GUI connection client object
    # connect(use_gui=True,width=1000, height=700)
    connect(use_gui=True,width=1920, height=1080)
    disable_real_time()

    enable_gravity()

    # Set camera pose to desired position and orientation
    set_camera_pose((0.0, -1.06, 1.5),(0,0,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    # Draw X, Y, Z axes
    draw_global_system()
    with HideOutput():
        # Load robot model given macro to urdf file and fix its base
        robot = load_model(HDT_MICHIGAN_URDF, fixed_base = False) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # Load floor model from urdf file and fix its base
        floor = load_model(os.path.join(cfg.ROOT_DIR, 'models/short_floor.urdf'), fixed_base=True)


    joints = get_arm_joints(robot, is_left=True, include_torso=False)
    set_joint_positions(robot, joints, init_conf)

    for t in range(0, 50):
        step_simulation()

    # p.setCollisionFilterPair(robot, robot, 37, 39, 0)

    # Load a block that we would like to use as our object of interest
    #block = load_model(BLOCK_URDF, fixed_base=True)
    # Set location and orientation of block in the simulation world
    #set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    # Get plant positions given the kind of placement of plants required
    plant_positions = envs[args.env]

    # Generate plants given positions
    plant_ids, plant_representations = generate_tall_plants(num_plants=len(plant_positions), positions=plant_positions, floor=floor)
    
    for t in range(0, 50):
        step_simulation()
    input("done")

    dump_world()

    # initialize deflection limit from arguments input
    deflection_limit = args.deflection_limit

    # Save the state of the world before beginning the planning process
    saved_world = p.saveState()
    
    # Create a BodyConf object with the initial configuration stored in it
    conf_i = BodyConf(robot, configuration=init_conf)
    # Create a BodyConf object with the final or end configuration stored in it
    conf_g = BodyConf(robot, configuration=goal_conf)
    
    # Moving arm from conf_i to conf_g and avoiding the block, floor. The plants are deflected within their limits to
    # reach the goal
    command = move_arm_conf2conf(robot, [floor, plant_ids[1]], plant_representations, deflection_limit,
                                 conf_i, conf_g)
    
    if (command is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return
    
    # Restore the state of the simulation after the planning process
    p.restoreState(saved_world)
    update_state()
    
    # # Save a video of the execution if required
    # if(args.video_filename != None):
    #     print("logging at ", args.video_filename)
    #     log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=args.video_filename)
    
    # command.execute_with_controls(robot, conf_i.joints, init_conf, plant_ids, plant_representations,
    #                                                         deflection_limit)
    # print('Quitting simulation')
    
    # # shutting down logger
    # if(args.video_filename != None):
    #     print("stopping logging")
    #     p.stopStateLogging(log_id)
    #input("wait for enter")
    disconnect()

if __name__ == '__main__':
    #cProfile.run('main()')
    main()
