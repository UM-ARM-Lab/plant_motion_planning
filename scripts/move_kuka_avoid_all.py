#l!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state,\
    disable_real_time, HideOutput, create_box, GREEN, BROWN, pairwise_collision, DRAKE_IIWA_URDF_EDIT
import pybullet as p
import numpy as np

def move_arm_conf2conf(robot, fixed, conf_i, conf_g):

    free_motion_fn = get_free_motion_gen(robot, fixed = (fixed), teleport=False)
    path, = free_motion_fn(conf_i, conf_g)

    return Command(path.body_paths)

init_conf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
goal_conf = (1.5, -1.0, 1.0, 1.5,0.0,0.0,0.0)

def main(display='execute'): # control | execute | step
    # connect(use_gui=True,width=1000, height=700)
    connect(use_gui=True)
    disable_real_time()

    # log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName="./rec1.mp4")

    draw_global_system()
    with HideOutput():
        robot = load_model(DRAKE_IIWA_URDF_EDIT) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # robot = load_model("./models/drake/iiwa_description/urdf/iiwa_polytope_collision.urdf") # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    plant_start_origin = p.getQuaternionFromEuler([0, 0, 0])

    ## Creating and placing a plant as an obstacle
    # plant = create_box(0.05,0.05,1,color=BROWN)
    plant = load_model('scripts/rigid_stem_rigid_branch.urdf', fixed_base=True)
    # plant_id = p.loadURDF("rigid_stem_rigid_branch.urdf", basePosition=[-0.5, -0.5, 0.0], baseOrientation=plant_start_origin,
    #                       useFixedBase=True)
    # plant = p.loadURDF('rigid_stem_rigid_branch.urdf')

    px = 0
    py = 0
    while(px < 0.1 and py > -0.1):
        px = np.random.rand() * (0.6) - 0.2
        py = np.random.rand() * (0.6) - 0.4

    set_pose(plant,Pose(Point(x=px,y=py,z=stable_z(plant,floor))))
    # set_pose(plant,Pose(Point(x=-0.3,y=0.1,z=stable_z(plant,floor))))

    set_default_camera(distance=1.8)
    dump_world()

    conf0 = BodyConf(robot, configuration=init_conf)
    conf0.assign()

    saved_world = WorldSaver()
    # command = plan(robot, block, fixed=[floor], teleport=False)

    conf_i = BodyConf(robot, configuration=init_conf)
    conf_g = BodyConf(robot, configuration=goal_conf)

    # print("collision with floor? ", pairwise_collision(robot, floor))
    # print("collision with plant? ", pairwise_collision(robot, plant))
    # print("collision with block? ", pairwise_collision(robot, block))

    ## Moving arm from conf_i to conf_g
    command = move_arm_conf2conf(robot, [floor, block], conf_i, conf_g)

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
    elif display == 'step':
        command.step()
    else:
        raise ValueError(display)

    print('Quit?')
    wait_if_gui()
    # p.stopStateLogging(log_id)
    disconnect()

if __name__ == '__main__':
    main()