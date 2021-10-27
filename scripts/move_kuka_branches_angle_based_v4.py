#l!/usr/bin/env python

from __future__ import print_function

import pybullet_data
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_ik_fn2, get_ik_fn_angle, get_free_motion_gen, get_holding_motion_gen,\
    get_free_motion_gen_with_angle_constraints, get_free_motion_gen_with_angle_constraints_v2, \
    get_free_motion_gen_with_angle_constraints_v4
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state,\
    disable_real_time, enable_real_time, HideOutput, create_box, GREEN, BROWN, pairwise_collision, \
    DRAKE_IIWA_URDF_EDIT, save_state, get_distance_fn
import pybullet as p
import numpy as np
from plant_motion_planning import representation
from .utils import set_random_poses, make_plant_responsive, set_random_pose

from datetime import datetime


def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g, teleport=False):

    start_state_id = save_state()

    # free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed=([block] + fixed), movable = movable,
    free_motion_fn = get_free_motion_gen_with_angle_constraints_v4(robot, start_state_id, fixed= fixed, movable = movable,
                                                                   deflection_limit = deflection_limit, teleport=teleport)
    # free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)

    num_attempts = 200

    for attempt in range(num_attempts):

        result = free_motion_fn(conf_i, conf_g)

        if result is None:
            continue
        else:
            path, = result
            return Command(path.body_paths)

    return None


init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
goal_conf = (-1.3871757013351371, 1.6063773991870438, 2.152853076950719, -1.0638334445672613, -0.1398096715085235,
             1.9391079682303638, -1.051507203470595)


def main(display='execute'): # control | execute | step

    # connect(use_gui=True,width=1000, height=700)
    connect(use_gui=True,width=1920, height=1080)
    # connect(use_gui=True)
    # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    disable_real_time()

    set_camera_pose((0.0, -1.06, 1.5),(0,0,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF_EDIT, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # robot = load_model("./models/drake/iiwa_description/urdf/iiwa_polytope_collision.urdf") # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=True)
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    ## Creating and placing a plant as an obstacle
    # plant = create_box(0.05,0.05,1,color=BROWN)
    # plant = load_model('my_scripts/rigid_stem_rigid_branch.urdf', fixed_base=True)
    plant1 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)
    plant2 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)
    plant3 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)
    plant4 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)
    plant5 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)

    # Difficult test case!
    # set_random_pose(plant1, floor, px = 0.4, py = 0.1)
    # set_random_pose(plant2, floor, px = 0.12, py = -0.15)

    # ENV1
    set_random_pose(plant1, floor, px = 0.4, py = 0.1)
    set_random_pose(plant2, floor, px = 0.12, py = -0.15)
    set_random_pose(plant3, floor, px = -0.25, py = 0.60)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    set_random_pose(plant4, floor, px = -0.25, py = 0.45)
    set_random_pose(plant5, floor, px = -0.05, py = 0.50)

    # ENV2
    # set_random_pose(plant1, floor, px = 0.15, py = -0.1)
    # set_random_pose(plant2, floor, px = -0.08, py = -0.15)
    # set_random_pose(plant3, floor, px = 0.25, py = -0.60)
    # # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.43)
    # set_random_pose(plant5, floor, px = -0.05, py = -0.48)

    # ENV3
    # set_random_pose(plant1, floor, px = 0.4, py = -0.25)
    # set_random_pose(plant2, floor, px = 0.25, py = -0.25)
    # set_random_pose(plant3, floor, px = 0.25, py = -0.60)
    # # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.40)
    # set_random_pose(plant5, floor, px = 0.40, py = -0.60)

    # input("")
    # exit()

    # ENV4
    # set_random_pose(plant1, floor, px = 0.15, py = -0.1)
    # set_random_pose(plant2, floor, px = -0.08, py = -0.15)
    # set_random_pose(plant3, floor, px = 0.25, py = -0.60)
    # # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.45)
    # set_random_pose(plant5, floor, px = -0.0, py = -0.43)

    # ENV5: NOT POSSIBLE AS MANUAL GOAL CONFIGURATION SET IS VIOLATING CONSTRAINTS
    # set_random_pose(plant1, floor, px = 0.15, py = -0.1)
    # set_random_pose(plant2, floor, px = -0.08, py = -0.15)
    # set_random_pose(plant3, floor, px = 0.25, py = -0.60)
    # # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.45)
    # set_random_pose(plant5, floor, px = 0.03, py = -0.30)

    # ENV6
    # set_random_pose(plant1, floor, px = 0.15, py = -0.1)
    # set_random_pose(plant2, floor, px = -0.08, py = -0.15)
    # set_random_pose(plant3, floor, px = 0.25, py = -0.60)
    # # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
    # set_random_pose(plant4, floor, px = 0.25, py = -0.45)
    # set_random_pose(plant5, floor, px = 0.4, py = -0.70)

    make_plant_responsive([plant1, plant2, plant3, plant4, plant5])
    # make_plant_responsive([plant1, plant2])

    dump_world()

    conf0 = BodyConf(robot, configuration=init_conf)
    conf0.assign()

    plant1_2angle_rep = representation.TwoAngleRepresentation(plant1, 1)
    plant2_2angle_rep = representation.TwoAngleRepresentation(plant2, 1)
    plant3_2angle_rep = representation.TwoAngleRepresentation(plant3, 1)
    plant4_2angle_rep = representation.TwoAngleRepresentation(plant4, 1)
    plant5_2angle_rep = representation.TwoAngleRepresentation(plant5, 1)

    deflection_limit = 0.50

    saved_world = p.saveState()

    conf_i = BodyConf(robot, configuration=init_conf)
    conf_g = BodyConf(robot, configuration=goal_conf)

    ## Moving arm from conf_i to conf_g
    # command = move_arm_conf2conf(robot, [floor, block], [plant2_2angle_rep, plant1_2angle_rep],
    #                              deflection_limit, conf_i, conf_g)

    command = move_arm_conf2conf(robot, [floor, block], [plant2_2angle_rep, plant1_2angle_rep, plant3_2angle_rep,
                                                         plant4_2angle_rep, plant5_2angle_rep], deflection_limit,
                                 conf_i, conf_g)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return

    # input("Path calculated! Press enter to continue...")

    distance_fn = get_distance_fn(robot, conf_g.joints, weights=None)
    alpha = 0.1
    cost_utils = [distance_fn, alpha]

    # saved_world.restore()
    p.restoreState(saved_world)
    update_state()
    # wait_if_gui('{}?'.format(display))

    log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,
                    fileName="../testing_data/angle_constraint_v4_1/video_recordings/env1_rec1.mp4")
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
        command.execute_with_movable_plant_angle_constraint(robot, block, [plant2, plant1, plant3, plant4, plant5],
                                                            [plant2_2angle_rep, plant1_2angle_rep,
                                                             plant3_2angle_rep, plant4_2angle_rep, plant5_2angle_rep],
                                                            deflection_limit, cost_utils)
        # command.execute_with_movable_plant_angle_constraint(robot, block, [plant2, plant1],
        #                                                     [plant2_2angle_rep, plant1_2angle_rep],
        #                                                     deflection_limit, cost_utils)
    else:
        raise ValueError(display)

    print('Quit?')
    wait_if_gui()
    p.stopStateLogging(log_id)
    disconnect()

if __name__ == '__main__':
    # main()
    # main(display='step')
    # main(display = "execute_with_movable_plant")
    main(display = "angle_step")
