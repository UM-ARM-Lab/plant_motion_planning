#l!/usr/bin/env python

from __future__ import print_function

import pybullet_data
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_ik_fn2, get_ik_fn_angle, get_free_motion_gen, get_holding_motion_gen,\
    get_free_motion_gen_with_angle_constraints, get_free_motion_gen_with_angle_constraints_v2, \
    get_free_motion_gen_with_angle_constraints_v4
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state, \
    disable_real_time, enable_real_time, HideOutput, create_box, GREEN, BROWN, pairwise_collision, \
    DRAKE_IIWA_URDF_EDIT, save_state, get_distance_fn, get_movable_joints, set_joint_positions
import pybullet as p
import numpy as np
from plant_motion_planning import representation
from .utils import set_random_poses, make_plant_responsive, set_random_pose, generate_plants, envs, step_sim

from datetime import datetime


def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g, teleport=False):

    start_state_id = save_state()

    # free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed=([block] + fixed), movable = movable,
    free_motion_fn = get_free_motion_gen_with_angle_constraints_v4(robot, start_state_id, fixed= fixed, movable = movable,
                                                                   deflection_limit = deflection_limit, teleport=teleport)
    # free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)

    num_attempts = 200

    path = []
    for attempt in range(num_attempts):

        result = free_motion_fn(conf_i, conf_g)

        if result is None:
            continue
        else:
            path, = result
            break
            # return Command(path.body_paths)

    # input("executing path...")

    p.restoreState(start_state_id)

    joints = get_movable_joints(robot)
    position_gains = 7 * [0.01]

    set_joint_positions(robot, joints, init_conf)
    p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, init_conf, positionGains=position_gains)
    for t in range(20):
        step_sim()

    # input("press enter to execute forward path!")
    print("press enter to execute forward path!")
    # print("***********************************")
    # print("path smoothed: ", path.body_paths[0].path)
    # print("***********************************")
    for q in path.body_paths[0].path:

        p.setJointMotorControlArray(robot, joints, p.POSITION_CONTROL, q, positionGains=position_gains)

        for t in range(11):
            step_sim()

        for e, b in enumerate(movable):
            b.observe()

            print("Deflection of plant %d: %f" % (e, b.deflection))

            if(b.deflection > deflection_limit):
                print("Error! Deflection limit exceeded!")
                import pickle
                with open('path_smoothed.pkl', 'wb') as f:
                    pickle.dump([path.body_paths[0].path], f)

                input("")
                # exit()


            # input("press enter to step...")


    # wait_if_gui("Press enter to continue...")
    exit()

    return Command(path.body_paths)

    return None


init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
goal_conf = (-1.3871757013351371, 1.6063773991870438, 2.152853076950719, -1.0638334445672613, -0.1398096715085235,
             1.9391079682303638, -1.051507203470595)


def main(display='execute'): # control | execute | step

    connect(use_gui=True,width=1000, height=700)
    # connect(use_gui=True,width=1920, height=1080)
    # connect(use_gui=True)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    disable_real_time()

    set_camera_pose((0.0, -1.06, 1.5),(0,0,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF_EDIT, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf', fixed_base=True)
    block = load_model(BLOCK_URDF, fixed_base=True)
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    # Get plant positions given the kind of placement of plants required
    plant_positions = envs["env1"]

    # Generate plants given positions
    plant_ids, plant_representations = generate_plants(num_plants=5, positions=plant_positions, floor=floor)

    dump_world()

    conf0 = BodyConf(robot, configuration=init_conf)
    conf0.assign()

    deflection_limit = 0.50

    saved_world = p.saveState()

    conf_i = BodyConf(robot, configuration=init_conf)
    conf_g = BodyConf(robot, configuration=goal_conf)

    # Moving arm from conf_i to conf_g
    command = move_arm_conf2conf(robot, [floor, block], plant_representations, deflection_limit,
                                 conf_i, conf_g)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return

    p.restoreState(saved_world)
    update_state()
    # wait_if_gui('{}?'.format(display))

    # log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,
    #                 fileName="../testing_data/angle_constraint_v4_1/video_recordings/env5/trial5.mp4")
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
        command.execute_with_movable_plant_angle_constraint(robot, block, plant_ids, plant_representations,
                                                            deflection_limit)
    else:
        raise ValueError(display)

    print('Quit?')
    # wait_if_gui()
    # p.stopStateLogging(log_id)
    disconnect()

if __name__ == '__main__':
    # main()
    # main(display='step')
    # main(display = "execute_with_movable_plant")
    main(display = "angle_step")
