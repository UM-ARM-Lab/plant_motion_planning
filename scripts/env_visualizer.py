#l!/usr/bin/env python

from __future__ import print_function

import pybullet_data
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_ik_fn2, get_ik_fn_angle, get_free_motion_gen, get_holding_motion_gen, \
    get_free_motion_gen_with_angle_constraints, get_free_motion_gen_with_angle_constraints_v2, \
    get_free_motion_gen_with_angle_constraints_v4
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, draw_pose, set_camera_pose, Pose, Point, Euler, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_if_gui, disconnect, DRAKE_IIWA_URDF, wait_if_gui, update_state, \
    disable_real_time, enable_real_time, HideOutput, create_box, GREEN, BROWN, pairwise_collision, \
    DRAKE_IIWA_URDF_EDIT, save_state, get_distance_fn
import pybullet as p
import numpy as np
from plant_motion_planning import representation
from .utils import set_random_poses, make_plant_responsive, set_random_pose

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
        # robot = load_model("./models/drake/iiwa_description/urdf/iiwa_polytope_collision.urdf") # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        floor = load_model('models/short_floor.urdf', fixed_base=True)
    block = load_model(BLOCK_URDF, fixed_base=True)
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    ## Creating and placing a plant as an obstacle
    # plant = create_box(0.05,0.05,1,color=BROWN)
    # plant = load_model('my_scripts/rigid_stem_rigid_branch.urdf', fixed_base=True)
    plant1 = load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True)
    plant2 = load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True)
    plant3 = load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True)
    plant4 = load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True)
    plant5 = load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True)


    pids = [plant1, plant2, plant3, plant4, plant5]

    # ENV6
    set_random_pose(plant1, floor, px = 0.098, py = -0.10)
    set_random_pose(plant2, floor, px = -0.005, py = -0.25)
    set_random_pose(plant3, floor, px = 0.1, py = -0.32)
    set_random_pose(plant4, floor, px = 0.14, py = -0.49)
    set_random_pose(plant5, floor, px = 0.4, py = -0.578)

    plant1_2angle_rep = representation.TwoAngleRepresentation(plant1, 1)
    plant2_2angle_rep = representation.TwoAngleRepresentation(plant2, 1)
    plant3_2angle_rep = representation.TwoAngleRepresentation(plant3, 1)
    plant4_2angle_rep = representation.TwoAngleRepresentation(plant4, 1)
    plant5_2angle_rep = representation.TwoAngleRepresentation(plant5, 1)

    make_plant_responsive([plant1, plant2, plant3, plant4, plant5])
    # make_plant_responsive([plant1, plant2, plant3, plant4])

    dump_world()

    # conf0 = BodyConf(robot, configuration=init_conf)
    # conf0.assign()

    conf1 = BodyConf(robot, configuration=goal_conf)
    conf1.assign()


    flag = 0
    movable = [plant1_2angle_rep, plant2_2angle_rep, plant3_2angle_rep, plant4_2angle_rep, plant5_2angle_rep]
    # movable = [plant1_2angle_rep, plant2_2angle_rep, plant3_2angle_rep, plant4_2angle_rep] #, plant5_2angle_rep]



    for t in range(1000):

        for pid in pids:
            plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = p.getJointState(pid, 0)
            plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = p.getJointState(pid, 1)
            p.applyExternalTorque(pid, linkIndex=1,
                                         torqueObj=[-200 * plant_rot_joint_displacement_x,
                                                    -200 * plant_rot_joint_displacement_y, 0],
                                         flags=p.WORLD_FRAME)

        p.stepSimulation()

    for t in range(1000):
        p.stepSimulation()

    print("================================================")
    for e, b in enumerate(movable):
        b.observe()

        print("Deflection of plant %d: %0.3f" % (e, b.deflection))

        if (b.deflection > 0):
            flag = 1

    if (flag == 1):
        print("Warning! End configuration is in contact with plants")

    print("================================================")

    wait_if_gui('Environment visualized...')
    disconnect()

if __name__ == '__main__':
    main()