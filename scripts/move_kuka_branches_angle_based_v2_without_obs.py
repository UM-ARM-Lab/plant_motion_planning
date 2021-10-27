#l!/usr/bin/env python

from __future__ import print_function

import pybullet_data
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_ik_fn2, get_ik_fn_angle, get_free_motion_gen, get_holding_motion_gen,\
    get_free_motion_gen_with_angle_constraints, get_free_motion_gen_with_angle_constraints_v2
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state,\
    disable_real_time, enable_real_time, HideOutput, create_box, GREEN, BROWN, pairwise_collision, \
    DRAKE_IIWA_URDF_EDIT, save_state
import pybullet as p
import numpy as np
from plant_motion_planning import representation

def move_arm(robot, block, fixed,teleport):
    grasp_gen = get_grasp_gen(robot, 'side_only')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport,num_attempts=1000)

    ## PROBLEMATIC
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, configuration=init_conf)

    saved_world = WorldSaver()

    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)

        if result1 is None:
            continue
        conf1, path2 = result1

        pose0.assign()

        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2

        return Command(path1.body_paths)

    return None

## Modified move_arm() function
def move_arm2(robot, block, force_limit, fixed, movable, teleport):
    grasp_gen = get_grasp_gen(robot, 'side_only')
    ik_fn = get_ik_fn2(robot, fixed=fixed, movable = movable, force_limit=force_limit, teleport=teleport,num_attempts=1000)

    ## PROBLEMATIC
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, configuration=init_conf)

    saved_world = WorldSaver()

    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)

        if result1 is None:
            continue
        conf1, path2 = result1

        pose0.assign()

        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2

        return Command(path1.body_paths)

    return None

def move_arm_with_angle_constraint(robot, block, plant, fixed, movable, deflection_limit, teleport):
    grasp_gen = get_grasp_gen(robot, 'side_only')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport,num_attempts=1000)
    # ik_fn = get_ik_fn_angle(robot, fixed=fixed, movable = movable, deflection_limit = deflection_limit, teleport=teleport,num_attempts=1000)

    free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed=([block] + fixed), movable = movable,
                                                                deflection_limit = deflection_limit, teleport=teleport)
    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, configuration=init_conf)

    saved_world = WorldSaver()

    for grasp, in grasp_gen(block):
        saved_world.restore()

        # Step 1. Use IK to get joint values
        result1 = ik_fn(block, pose0, grasp)

        if result1 is None:
            continue
        conf1, path2 = result1

        # conf1 holds goal state's joint values
        ####################################

        pose0.assign()

        # Step 2. Reset joint values to initial configuration
        conf0.assign()
        #####################################

        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2

        return Command(path1.body_paths)

    return None


def move_arm_conf2conf(robot, fixed, movable, deflection_limit, conf_i, conf_g, teleport=False):

    # free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed=([block] + fixed), movable = movable,
    free_motion_fn = get_free_motion_gen_with_angle_constraints_v2(robot, fixed= fixed, movable = movable,
                                                                   deflection_limit = deflection_limit, teleport=teleport)
    # free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)

    num_attempts = 100

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

    connect(use_gui=True,width=1000, height=700)
    # connect(use_gui=True)
    disable_real_time()

    set_camera_pose((1.5,1.0,1.5),(0,0,0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    # log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,
    #                              fileName="./rec1_unedited.mp4")

    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF_EDIT, fixed_base = True) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # robot = load_model("./models/drake/iiwa_description/urdf/iiwa_polytope_collision.urdf") # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    plant_start_origin = p.getQuaternionFromEuler([0, 0, 0])

    ## Creating and placing a plant as an obstacle
    # plant = create_box(0.05,0.05,1,color=BROWN)
    # plant = load_model('my_scripts/rigid_stem_rigid_branch.urdf', fixed_base=True)
    plant1 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)
    # plant2 = load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True)

    # plant = load_model('my_scripts/plant_multi_dof_model_edit.urdf', fixed_base=True)
    # plant_id = p.loadURDF("rigid_stem_rigid_branch.urdf", basePosition=[-0.5, -0.5, 0.0], baseOrientation=plant_start_origin,
    #                       useFixedBase=True)
    # plant = p.loadURDF('rigid_stem_rigid_branch.urdf')

    px = 0.0
    py = 0.0
    while(px < 0.1 and py > -0.1):
        px = np.random.rand() * (0.6) - 0.2
        py = np.random.rand() * (0.6) - 0.4
    set_pose(plant1, Pose(Point(x=px, y=py, z=stable_z(plant1, floor))))
    # set_pose(plant,Pose(Point(x=-0.3,y=0.1,z=stable_z(plant,floor))))

    # set_pose(plant2, Pose(Point(x=0.3, y=-0.1, z=stable_z(plant2, floor))))


    p.setJointMotorControlArray(plant1, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
                                forces=[1e-1, 1e-1])  # make plant responsive to external force

    # p.setJointMotorControlArray(plant2, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
    #                             forces=[1e-1, 1e-1])  # make plant responsive to external force

    # set_default_camera(distance=1.5)
    dump_world()

    conf0 = BodyConf(robot, configuration=init_conf)
    conf0.assign()

    # Object that stores the two angle representation of a plant
    plant1_2angle_rep = representation.TwoAngleRepresentation(plant1, 1)
    # plant2_2angle_rep = representation.TwoAngleRepresentation(plant2, 1)

    ## Deflection limit (in radians) - applied on the plants by the robot
    deflection_limit = 0.75

    saved_world = WorldSaver()
    # command = plan(robot, block, fixed=[floor], teleport=False)
    # command = move_arm(robot, block, fixed=[floor,plant], teleport=False)
    # command = move_arm(robot, block, fixed=[floor], teleport=False)
    # command = move_arm_with_angle_constraint(robot, block, [plant1], fixed=[floor],
    #                                          movable = [plant1_2angle_rep],
    #                                          deflection_limit = deflection_limit, teleport=False)

    # print("collision with floor? ", pairwise_collision(robot, floor))
    # print("collision with plant? ", pairwise_collision(robot, plant))
    # print("collision with block? ", pairwise_collision(robot, block))

    conf_i = BodyConf(robot, configuration=init_conf)
    conf_g = BodyConf(robot, configuration=goal_conf)

    ## Moving arm from conf_i to conf_g
    command = move_arm_conf2conf(robot,[floor, block], [plant1_2angle_rep], deflection_limit, conf_i, conf_g)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print("*********************************************************")
        return

    saved_world.restore()
    update_state()
    wait_if_gui('{}?'.format(display))
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.005)
    elif display == 'execute_with_movable_plant':
        command.refine(num_steps = 10).execute_with_movable_plant(robot, plant1,time_step = 0.005)
    elif display == 'step':
        command.step()
    elif display == 'angle_step':
        command.execute_with_movable_plant_angle_constraint(robot, [plant1],
                                                            [plant1_2angle_rep])
    else:
        raise ValueError(display)

    print('Quit?')
    wait_if_gui()
    # p.stopStateLogging(log_id)
    disconnect()

if __name__ == '__main__':
    # main()
    # main(display='step')
    # main(display = "execute_with_movable_plant")
    main(display = "angle_step")
