#l!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_ik_fn2, get_free_motion_gen, get_holding_motion_gen
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state,\
    disable_real_time, HideOutput, create_box, GREEN, pairwise_collision


def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'side_only')
    ik_fn = get_ik_fn2(robot, fixed=fixed, teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)

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
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        return Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
    return None


def move_arm(robot, block, fixed,teleport):
    grasp_gen = get_grasp_gen(robot, 'side_only')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport,num_attempts=100)

    print("==============================")
    print("fixed: ", fixed)
    print("==============================")

    ## PROBLEMATIC
    free_motion_fn = get_free_motion_gen(robot, fixed=[block] + fixed, teleport=teleport)

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

def move_arm_conf2conf(robot, block, fixed, conf_i, conf_g):

    free_motion_fn = get_free_motion_gen(robot, fixed = ([block] + fixed), teleport=False)
    path, = free_motion_fn(conf_i, conf_g)

    return Command(path.body_paths)

init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

## goal config that causes colllision of end-effector with the plant
# goal_conf = (2.0, 0.0, 0.0, 1.4,0.0,0.0,0.0)

## goal config that doest not cause a colllision of end-effector with the plant
goal_conf = (2.0, -1.5, 0.0, 1.4,0.0,0.0,0.0)

# goal_conf = (-1.1729920777758336, 1.4589943767965057, -0.5616478337672394, 0.801648500090313, -0.3073797715302726, -2.010681695296876, 2.4245183873058753)
# goal_conf = (1.7621129623536114, -1.1253366511988856, -1.4983783078208517, -0.801219761743335, -2.6648383747580766, -2.046016835591183, 1.9713332585462995)
# goal_conf = (-1.3783358548929636, 1.1412368617511968, -1.4601638849763252, 0.8020818472093942, 0.4459277059091266, -2.0381584498394267, 1.9879204280826481)
# goal_conf = (2.0597890390562874, -1.4950103178373757, -0.3306144578216243, -0.787732589913809, -0.499377857194417, 2.0576522335542244, -0.6151893819113333)

def main(display='execute'): # control | execute | step
    connect(use_gui=True)
    disable_real_time()
    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.6),Euler(yaw=1.57)))

    ## Creating and placing a plant as an obstacle
    plant = create_box(0.05,0.05,1,color=GREEN)
    set_pose(plant,Pose(Point(x=0.5,y=0,z=stable_z(plant,floor))))

    set_default_camera(distance=2)
    dump_world()

    conf0 = BodyConf(robot, configuration=goal_conf)
    conf0.assign()

    saved_world = WorldSaver()
    # command = plan(robot, block, fixed=[floor], teleport=False)
    # command = move_arm(robot, block, fixed=[floor] + [plant], teleport=True)

    conf_i = BodyConf(robot, configuration=init_conf)
    conf_g = BodyConf(robot, configuration=goal_conf)

    print("collision with floor? ", pairwise_collision(robot, floor))
    print("collision with plant? ", pairwise_collision(robot,plant))
    print("collision with block? ", pairwise_collision(robot, block))

    print("********************************************************************")

    ## Moving arm from conf_i to conf_g
    # command = move_arm_conf2conf(robot,block,[floor,plant],conf_i, conf_g)

    # if (command is None) or (display is None):
    #     print('Unable to find a plan!')
    #     print("*********************************************************")
    #     return
    #
    # saved_world.restore()
    # update_state()
    # wait_if_gui('{}?'.format(display))
    # if display == 'control':
    #     enable_gravity()
    #     command.control(real_time=False, dt=0)
    # elif display == 'execute':
    #     command.refine(num_steps=10).execute(time_step=0.005)
    # elif display == 'step':
    #     command.step()
    # else:
    #     raise ValueError(display)

    print('Quit?')
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()