import numpy as np
import pybullet
import pybullet as p
# from pybullet_tools.utils import set_pose, Pose, Point, stable_z, step_simulation
import pybullet_tools.utils as pyb_tools_utils

def set_random_pose(plant_id, floor, px = None, py = None):

    if px is None and py is None:
        px = 0.0
        py = 0.0
        while(px < 0.1 and py > -0.1):
            px = np.random.rand() * (0.6) - 0.2
            py = np.random.rand() * (0.6) - 0.4

    pyb_tools_utils.set_pose(plant_id, pyb_tools_utils.Pose(pyb_tools_utils.Point(x=px, y=py,
                                                                                  z=pyb_tools_utils.stable_z(plant_id, floor))))

def set_random_poses(plant_ids = [], floor = 0):

    for plant_id in plant_ids:
        set_random_pose(plant_id, floor)

def make_plant_responsive(plant_ids = []):

    for plant_id in plant_ids:
        p.setJointMotorControlArray(plant_id, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
                                forces=[1e-1, 1e-1])  # make plant responsive to external force

def step_sim():

    # plant_id = [3, 4]
    plant_id = [3, 4, 5, 6, 7]
    # plant_id = [3]

    for pid in plant_id:
        plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
        plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
        pybullet.applyExternalTorque(pid, linkIndex=1,
                                     torqueObj=[-200 * plant_rot_joint_displacement_x,
                                                -200 * plant_rot_joint_displacement_y, 0],
                                     flags=pybullet.WORLD_FRAME)

    for t in range(200):
        pyb_tools_utils.step_simulation()
        # pyb_tools_utils.wait_for_duration(0.02)