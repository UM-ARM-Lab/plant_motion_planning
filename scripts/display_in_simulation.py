"""
This file was used in testing to visualize the single stemmed plant.
"""

import os
import time
import math
import pybullet as p
import pybullet_data
from plant_motion_planning import cfg, representation


trail_length_limit = math.pi / 2
physicsClient = p.connect(p.GUI, options="--width=1200 --height=400")  # or p.DIRECT for non-graphical version

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)

# Load the model of the checkerboard plane to serve as a "floor" for simulation
plane_id = p.loadURDF("plane.urdf")

# The initial orientation of the plant
plant_start_ori = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
# Load a model of the plant along with position and orientation
plant_id = p.loadURDF(os.path.join(cfg.ROOT_DIR, "./urdf/plant_multi_dof_model_original.urdf"),
                      basePosition=[0, 0, 0.5], baseOrientation=plant_start_ori, useFixedBase=True)

# Make the plant responsive to external forces
p.setJointMotorControlArray(plant_id, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
                            forces=[1e-1, 1e-1])

################## Drawing the axes for all frames of interest ################

# Drawing world coordinate axes
utils.draw_frame_axes(start = (0,0,0), length = 50)
# Drawing the axes on pot
utils.draw_frame_axes(start = (0,0,0), length = 3, parentid = plant_id)
# Drawing the axes on plant
utils.draw_frame_axes(start = (0,0,-0.4), length = 2, parentid = plant_id, parentlinkid = 1)
# Drawing the axes on v1 link (invisible link connecting 2 joints)
utils.draw_frame_axes(start = (0,0,0), length = 2, parentid = plant_id, parentlinkid = 0)

##################################################################################

plant1 = representation.TwoAngleRepresentation(plant_id, 1)

text_ori = p.getQuaternionFromEuler([1.57, 0.0, 0.0])

# p.addUserDebugText("hello world", [1, 1, 1], textColorRGB=[0, 0, 0], textOrientation=text_ori)

t = 0
while True:
    plant1.observe()
    print(f"plant {plant1.bid} deflection magnitude {plant1.deflection} | radians orientation {plant1.orientation} radians")
    # print(f"deflection {plant1.deflection} orientation {plant1.orientation}")
    # print(f"orientation {plant1.orientation}")


    # plant reaction torque
    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = p.getJointState(plant_id, 0)
    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = p.getJointState(plant_id, 1)

    # running_sum_disp_x = running_sum_disp_x + plant_rot_joint_displacement_x
    # running_sum_disp_y = running_sum_disp_y + plant_rot_joint_displacement_y

    # p.applyExternalTorque(plant_id, linkIndex=1,
    #                       torqueObj=[-200 * plant_rot_joint_displacement_x + (-1) * running_sum_disp_x, -200 * plant_rot_joint_displacement_y + (-1) * running_sum_disp_y, 0],
    #                       flags=p.WORLD_FRAME)

    p.applyExternalTorque(plant_id, linkIndex=1,
                          torqueObj=[-200 * plant_rot_joint_displacement_x, -200 * plant_rot_joint_displacement_y, 0],
                          flags=p.WORLD_FRAME)

    if(t == 0):
        t = 1
    else:
        t = 0

    text_id = []

    # # Display text in simulation
    # text_id.append(p.addUserDebugText("value: " + str(t), [1, 1, 1], textColorRGB=[0, 0, 0]))
    # text_id.append(p.addUserDebugText("value: " + str(t), [1, 1, 0.8], textColorRGB=[0, 0, 0]))

    p.stepSimulation()
    time.sleep(0.01)

    for tid in text_id:
        p.removeUserDebugItem(tid)
