#l!/usr/bin/env python

"""
This program finds a path between two configurations of a robotic arm by deflecting plants within their limits and
without colliding with other objects.

This program only deals with plants with single stems. Refer to v6 of our method for a multi-branch implementation.

"""

from __future__ import print_function

import os

import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, Command, get_free_motion_gen_with_angle_constraints_v4
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, update_state, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT, save_state
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


# def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g, teleport=False):
#
#     start_state_id = save_state()
#
#     # free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed=([block] + fixed), movable = movable,
#     free_motion_fn = get_free_motion_gen_with_angle_constraints_v4(robot, start_state_id, fixed= fixed, movable = movable,
#                                                                    deflection_limit = deflection_limit, teleport=teleport)
#     # free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)
#
#     num_attempts = 200
#
#     path = []
#     for attempt in range(num_attempts):
#
#         result = free_motion_fn(conf_i, conf_g)
#
#         if result is None:
#             continue
#         else:
#             path, = result
#             break
#             # return Command(path.body_paths)
#
#     # input("executing path...")
#
#     p.restoreState(start_state_id)
#
#     joints = get_movable_joints(robot)
#     position_gains = 7 * [0.01]
#
#     set_joint_positions(robot, joints, init_conf)
#     p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, init_conf, positionGains=position_gains)
#     for t in range(20):
#         step_sim()
#
#     log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,
#                     fileName="./rec_env2.mp4")
#     text_id = []
#     p.addUserDebugText("Deflection limit: %0.3f rad" % (deflection_limit), [-1.95, 1, 1.05],
#                               textColorRGB=[1, 0, 0])
#     for e, b in enumerate(movable):
#         b.observe()
#         text_id.append(p.addUserDebugText("Deflection of plant " + str(e + 1) + ": " + str(b.deflection),
#                                                      [-2 - 0.2 * e, 1, 1 - 0.2 * e], textColorRGB=[0, 0, 0]))
#
#     # input("press enter to execute forward path!")
#     print("press enter to execute forward path!")
#     # print("***********************************")
#     # print("path smoothed: ", path.body_paths[0].path)
#     # print("***********************************")
#     for q in path.body_paths[0].path:
#
#         p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, q, positionGains=position_gains)
#
#         for t in range(11):
#             step_sim()
#
#         for e, b in enumerate(movable):
#             b.observe()
#
#             p.addUserDebugText("Deflection of plant " + str(e + 1) + ": %.3f rad" % (b.deflection),
#                                       [-2 - 0.07 * e, 1, 1 - 0.07 * e], textColorRGB=[0, 0, 0],
#                                       replaceItemUniqueId=text_id[e])
#
#             print("Deflection of plant %d: %f" % (e, b.deflection))
#
#             if(b.deflection > deflection_limit):
#                 print("Error! Deflection limit exceeded!")
#                 import pickle
#                 with open('path_smoothed.pkl', 'wb') as f:
#                     pickle.dump([path.body_paths[0].path], f)
#
#                 input("")
#                 # exit()
#
#
#             # input("press enter to step...")
#
#
#     # wait_if_gui("Press enter to continue...")
#     p.stopStateLogging(log_id)
#     exit()
#
#     return Command(path.body_paths)
#
#     return None


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
    connect(use_gui=True,width=1920, height=1080)
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

    # Get plant positions given the kind of placement of plants required
    plant_positions = envs[args.env]

    # Generate plants given positions
    plant_ids, plant_representations = generate_tall_plants(num_plants=len(plant_positions), positions=plant_positions, floor=floor)

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
    # command = move_arm_conf2conf(robot, [floor, block], plant_representations, deflection_limit,
    #                              conf_i, conf_g)
    command = move_arm_conf2conf(robot, [floor, block], [], deflection_limit,
                                 conf_i, conf_g)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return

    # Restore the state of the simulation after the planning process
    p.restoreState(saved_world)
    update_state()

    # Save a video of the execution if required
    if(args.video_filename != None):
        log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=args.video_filename)

    # Poll the value of display and execute accordingly
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.005)
    elif display == 'execute_with_movable_plant':
        command.refine(num_steps = 10).execute_with_movable_plant(robot, plant1, time_step = 0.005)
    elif display == 'step':
        command.step()
    elif display == 'angle_step':
        command.execute_with_controls(robot, init_conf, block, plant_ids, plant_representations,
                                                            deflection_limit)
    else:
        raise ValueError(display)

    print('Quitting simulation')

    # shutting down logger
    if(args.video_filename != None):
        p.stopStateLogging(log_id)
    disconnect()

if __name__ == '__main__':
    main(display = "angle_step")
