import os
import time
import math
import pybullet as p
import pybullet_data
import numpy as np
from plant_motion_planning import utils, cfg, representation
from pybullet_tools.utils import unit_point, unit_quat, draw_pose


trail_length_limit = math.pi / 2
physicsClient = p.connect(p.GUI, options="--width=1200 --height=400")  # or p.DIRECT for non-graphical version

p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, 0)


plane_id = p.loadURDF("plane.urdf")
plant_start_ori = p.getQuaternionFromEuler([0.0, 0.0, 0.0])
plant_id = p.loadURDF(os.path.join(cfg.ROOT_DIR, "./urdf/plant_multi_dof_model.urdf"), basePosition=[0, 0, 0.5], baseOrientation=plant_start_ori,
                      useFixedBase=True)
# arm_id = p.loadURDF("kuka_iiwa_interface/victor_description/urdf/victor.urdf", basePosition=[0, 3, 0],
#                     baseOrientation=p.getQuaternionFromEuler([0, 0, -math.pi / 4]), useFixedBase=1)
p.setJointMotorControlArray(plant_id, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
                            forces=[1e-1, 1e-1])  # make plant responsive to external force

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

# TODO find out what joint id, link id is for the joint that we want
plant1 = representation.TwoAngleRepresentation(plant_id, 1)


# running_sum_disp_x = 0.0
# running_sum_disp_y = 0.0

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

    p.stepSimulation()
    time.sleep(0.01)
