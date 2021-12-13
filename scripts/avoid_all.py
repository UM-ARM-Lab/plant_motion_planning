#l!/usr/bin/env python

from __future__ import print_function

import os

import numpy as np
import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, Command, get_free_motion_gen, get_free_motion_gen_with_controls
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, wait_if_gui, update_state, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT, get_movable_joints
import pybullet as p
from plant_motion_planning.utils import step_sim, \
    load_plant_from_urdf, generate_random_plant

import argparse


# Argument parser
parser = argparse.ArgumentParser(description='')
parser.add_argument("--video_filename", type=str, help="File name of video file to record planning with full path",
                    default=None)
parser.add_argument("--deflection_limit", type=float, help="Maximum amount of deflection to be permitted", default=None)
parser.add_argument("--plant_filename", type=str, help="Path to locations of plant model files",
                    default=None)
args = parser.parse_args()

if args.deflection_limit is None:
    print("Error!! All data must be provided")
    exit()


def move_arm_conf2conf(robot, fixed, conf_i, conf_g):
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

    free_motion_fn = get_free_motion_gen_with_controls(robot, fixed=fixed, teleport=False)

    for num_attempts in range(200):

        path_data = free_motion_fn(conf_i, conf_g)

        if(path_data is not None and path_data[0] is not None):
            path = path_data[0]
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
    cli = connect(use_gui=True,width=1000, height=700)
    # connect(use_gui=True,width=1920, height=1080)
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
    # num_branches_per_stem = 1
    # total_num_vert_stems = 2
    # total_num_extensions = 1
    # base_pos_xy = [np.random.uniform(low=0, high=0.35), np.random.uniform(low=-0.5, high=0.0)]
    # # base_pos_xy = [0.6, 0.6]
    # plant_id, plant_rep, joint_list, base_id = generate_random_plant(num_branches_per_stem, total_num_vert_stems, total_num_extensions,
    #                                             base_pos_xy, physicsClientId=cli, save_plant=True)

    # Load a plant from a previously generated random plant that has then been stored in a urdf file
    plant_id, plant_rep, joint_list, base_id = load_plant_from_urdf(os.path.join(args.plant_filename, "plant.urdf"),
                                               os.path.join(args.plant_filename, "plant_params.pkl"))

    dump_world()

    # initialize deflection limit from arguments input
    deflection_limit = args.deflection_limit

    # Save the state of the world before beginning the planning process
    saved_world = p.saveState()

    # Create a BodyConf object with the initial configuration stored in it
    conf_i = BodyConf(robot, configuration=init_conf)
    # Create a BodyConf object with the final configuration stored in it
    conf_g = BodyConf(robot, configuration=goal_conf)

    # Moving arm from conf_i to conf_g and avoiding the block, floor and plant, thereby avoiding all objects
    command = move_arm_conf2conf(robot, [floor, block, plant_id], conf_i, conf_g)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return

    # Restore the state of the simulation after the planning process
    p.restoreState(saved_world)
    update_state()

    # Save a video of the execution if required
    if (args.video_filename != None):
        log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName=args.video_filename)

    # Poll the value of display and execute accordingly
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.005)
    elif display == 'step':
        command.step()
    elif display == 'angle_step':
        command.execute_with_controls_v2(robot, init_conf, block, plant_id, [plant_rep],
                                         deflection_limit)
    else:
        raise ValueError(display)

    print('Quitting simulation')

    # shutting down logger
    if(args.video_filename != None):
        p.stopStateLogging(log_id)
    disconnect()


if __name__ == '__main__':
    main("angle_step")