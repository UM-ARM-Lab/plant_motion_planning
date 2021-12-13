#l!/usr/bin/env python

from __future__ import print_function

import os

import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf
from plant_motion_planning.pybullet_tools.utils import connect, dump_world, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, wait_if_gui, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT
import pybullet as p
from plant_motion_planning.utils import make_plant_responsive, generate_tall_plants


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

    # Connect to GUI environment
    connect(use_gui=True,width=1000, height=700)
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
    # set_pose(block, Pose(Point(x=-0.4, y=0.2, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=0.4,y=-0.4,z=0.45),Euler(yaw=1.57)))

    # (X, Y) positions of each plant
    positions = [
    [0.15, -0.10],
    [0.15, -0.20],
    [-0.10, -0.10],
    [-0.10, -0.20],
    [0.15, -0.32],
    [-0.10, -0.32],
    [-0.10, -0.42],
    [-0.1, -0.54],
    [0.15, -0.76],
    [0.15, -0.66],
    [-0.1, -0.76],
    [-0.1, -0.66],
    [0.40, -0.10],
    [0.40, -0.20],
    [0.15, -0.54],
    [0.40, -0.32],
    [0.47, -0.54],
    [0.40, -0.76],
    [0.40, -0.66],
    [0.55, -0.10],
    [0.55, -0.20],
    [0.55, -0.32],
    [0.55, -0.42],
    [0.55, -0.54],
    [0.55, -0.76],
    [0.55, -0.66],
    [-0.25, -0.10],
    [-0.25, -0.20],
    [-0.25, -0.32],
    ]

    # Generate tall plants given number of plants, positions and their base (floor)
    pids, movable = generate_tall_plants(len(positions), positions, floor)

    # Make the plants responsive
    make_plant_responsive(pids)
    # make_plant_responsive([plant1, plant2, plant3, plant4])

    dump_world()

    # Assign final goal configuration to Arm
    conf1 = BodyConf(robot, configuration=goal_conf)
    conf1.assign()

    flag = 0

    # Simulate environment for sufficient time for the plants to come to rest
    for t in range(1000):

        # Apply external torque to plants as a reaction to deflections
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

    # Measure the deflection of each plant and alert if there is a violation in constraints at the end configuration
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