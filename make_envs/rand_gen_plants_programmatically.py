# from random import random

import random

import numpy as np
import pybullet as p
import pybullet_data
import time

from pybullet_tools.utils import connect

p.connect(p.GUI)
# p.connect(p.GUI, options="--width=1920 --height=1080")
# connect(use_gui=True,width=1920, height=1080)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


# Remove debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

stem_base_spacing = 0.1

# Variables to keep track of indices
current_index = 0
main_stem_index = 0

stem_half_height = {}



def create_stem_element(stem_half_length, stem_half_width):

    stem_half_height[current_index] = np.random.uniform(low=0.3, high=0.7)

    col_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[stem_half_length, stem_half_width, stem_half_height[current_index]],
        collisionFramePosition=[0, 0, stem_base_spacing + stem_half_height[current_index]]
    )
    vis_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[stem_half_length, stem_half_width, stem_half_height[current_index]],
    )

    col_v1_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0, 0, 0],
        collisionFramePosition=[0, 0, 0.0]
    )
    vis_v1_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[0, 0, 0]
        # collisionFramePosition=[0, 0, 0]
    )

    return [col_v1_id, col_stem_id], [vis_v1_id, vis_stem_id]

def intersection_with_others(x, y, history, tolerance=0.03):
    if(len(history) == 0):
        history.append((x,y))
        return False
    else:

        for prev in history:
            if(np.abs(x - prev[0]) < tolerance and np.abs(y - prev[1]) < tolerance):
                return True

        history.append((x, y))
        return False


def create_plant_params():

    global current_index, main_stem_index

    base_half_width = np.random.uniform(low=0.15,high=0.5)
    base_half_length = np.random.uniform(low=0.15,high=0.5)
    base_half_height = np.random.uniform(low=0.15,high=1.0)

    col_base_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[base_half_width, base_half_length, base_half_height]
    )
    vis_base_id = -1

    # Base position and orientation
    base_pos = [0, 0, base_half_height]
    base_ori = [0, 0, 0, 1]

    # Base mass - Keep high to simulate a fixed base
    base_mass = 100000

    ###############

    stem_half_length = 0.1
    stem_half_width = 0.1
    # stem_half_height = np.random.uniform(low=0.3, high=0.7)

    link_Masses = []
    linkCollisionShapeIndices = []
    linkVisualShapeIndices = []

    linkPositions = []
    linkOrientations = []
    linkInertialFramePositions = []
    linkInertialFrameOrientations = []
    indices = []
    jointTypes = []
    axis = []

    total_num_stems = 2
    history = []

    exit_out = False
    ith_stem = 0
    num_branches_per_stem = 0
    branch_count = 1
    total_num_vert_stems = 2
    total_num_extensions = 1
    num_extensions = total_num_extensions + 1

    while(exit_out == False):


        if(ith_stem == 0):
            # Create a stem at the center of the base if its the 0th stem (first stem on the base)
            v1_pos = [0, 0, base_half_height]
            v1_ori = p.getQuaternionFromEuler((0, 0, 0.0))
            vert_stem_count = 1
            ith_stem = 1

            # Increment by 2 as 2 links have been added with the creating of a stem - v1 and the stem itself
            current_index = current_index + 2
            indices = indices + [main_stem_index, current_index - 1]
            main_stem_index = current_index

        elif(branch_count <= num_branches_per_stem):


            if(num_extensions <= total_num_extensions):
                v1_pos = [0, 0, stem_base_spacing + (2 * stem_half_height[current_index])]

                if(random.random() < 0.5):
                    roll = np.random.uniform(low=-0.5,high=0.5)
                    pitch = 0.0
                else:
                    pitch = np.random.uniform(low=-0.5,high=0.5)
                    roll = 0.0

                v1_ori = p.getQuaternionFromEuler((roll, pitch, 0.0))

                indices = indices + [current_index, current_index + 1]
                current_index = current_index + 2

                if(num_extensions == total_num_extensions):
                    branch_count = branch_count + 1

                num_extensions = num_extensions + 1


            else:
                num_extensions = 1

                while(1):
                    if(random.random() < 0.5):
                        x = random.choice([0.2, -0.2])
                        y = np.random.uniform(low=0.0, high=0.2)
                    else:
                        y = random.choice([0.2, -0.2])
                        x = np.random.uniform(low=0.0, high=0.2)

                    if(intersection_with_others(x,y, history, tolerance=0.2)):
                        continue
                    else:
                        # branch_count = branch_count + 1
                        break

                yaw = np.random.uniform(low=-0.5, high=0.5)

                if(np.abs(x) > np.abs(y)):
                    if(x > 0):
                        pitch = np.random.uniform(low=0.7, high=1.5)
                    else:
                        pitch = np.random.uniform(low=-1.5, high=-0.7)
                    roll = 0.0
                else:
                    if (y > 0):
                        roll = np.random.uniform(low=-1.5, high=-0.7)
                    else:
                        roll = np.random.uniform(low=0.7, high=1.5)

                    pitch = 0.0

                v1_pos = [x, y, np.random.uniform(low=stem_base_spacing,
                                                  high=stem_base_spacing + 2 * stem_half_height[main_stem_index])]

                v1_ori = p.getQuaternionFromEuler((roll, pitch, yaw))

                current_index = current_index + 2
                indices = indices + [main_stem_index, current_index - 1]

        elif(vert_stem_count == total_num_vert_stems):
            # input("chk2")
            exit_out = True
            break
        else:
            # Stack this stem on top of the old stem vertically
            vert_stem_count = vert_stem_count + 1
            branch_count = 1
            history = []

            v1_pos = [0, 0, stem_base_spacing + (2 * stem_half_height[main_stem_index])]

            if(random.random() < 0.5):
                pitch = np.random.uniform(low=-0.4, high=0.4)
                roll = 0.0
            else:
                roll = np.random.uniform(low=-0.4, high=0.4)
                pitch = 0.0

            yaw = np.random.uniform(low=-0.4, high=0.4)
            v1_ori = p.getQuaternionFromEuler((roll, pitch, yaw))

            current_index = current_index + 2
            indices = indices + [main_stem_index, current_index - 1]
            main_stem_index = current_index


        link_mass = [0, 10]
        col_stem_id, vis_stem_id = create_stem_element(stem_half_length, stem_half_width)

        # link_Masses  = link_Masses + [link_mass, link_mass]
        link_Masses  = link_Masses + link_mass
        linkCollisionShapeIndices = linkCollisionShapeIndices + col_stem_id
        linkVisualShapeIndices = linkVisualShapeIndices + vis_stem_id

        linkPositions = linkPositions + [v1_pos, [0, 0, 0]]
        linkOrientations = linkOrientations + [v1_ori, [0, 0, 0, 1]]

        linkInertialFramePositions = linkInertialFramePositions + [[0, 0, 0], [0, 0, 3]]
        linkInertialFrameOrientations = linkInertialFrameOrientations + [[0, 0, 0, 1], [0, 0, 0, 1]]

        jointTypes = jointTypes + [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
        axis = axis + [[1, 0, 0], [0, 1, 0]]

    return [base_mass, col_base_id, vis_base_id, base_pos, base_ori], [link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions,
                         linkOrientations, linkInertialFramePositions, linkInertialFrameOrientations, indices,
                         jointTypes, axis]



base_params, stems_params = create_plant_params()

base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params

link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
    linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params

print("base parameters: ", base_params)
print("stem parameters: ", stems_params)

# input("")


## TEMP CHANGE

# base_mass = 100000
#
# base_half_width = np.random.uniform(low=0.15, high=0.5)
# base_half_length = np.random.uniform(low=0.15, high=0.5)
# base_half_height = np.random.uniform(low=0.15, high=1.0)
#
# col_base_id = p.createCollisionShape(
#     p.GEOM_BOX, halfExtents=[base_half_length, base_half_width, base_half_height]
# )
# vis_base_id = -1
#
# # Base position and orientation
# base_pos = [0, 0, base_half_height]
# base_ori = [0, 0, 0, 1]
#
# col_stem1_id, vis_stem1_id = create_stem_element(0.1, 0.1)
#
# link_Masses = [0.1, 0.1]
# # linkCollisionShapeIndices = [col_v1_id, col_stem_id, col_v2_id, col_stem_id1]
# # linkVisualShapeIndices = [vis_v1_id, vis_stem_id, vis_v2_id, vis_stem_id1]
# linkCollisionShapeIndices = col_stem1_id
# linkVisualShapeIndices = vis_stem1_id
# linkPositions = [[0, 0, base_half_height], [0, 0, 0.0]]
# linkOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
# linkInertialFramePositions = [[0, 0, 0], [0, 0, 3]]
# linkInertialFrameOrientations = [[0, 0, 0, 1], [0, 0, 0, 1]]
# indices = [0, 1]
# # indices = [-1, 0, 1, 2]
# # jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
# jointTypes = [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
# axis = [[1, 0, 0], [0, 1, 0]]
#
#
# print("Base params: ", base_mass, col_base_id, vis_base_id, base_pos, base_ori)
# print("Stem_params: ", link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations,
#       linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis)

##


base_id = p.createMultiBody(
    base_mass, col_base_id, vis_base_id, base_pos, base_ori,
    linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
    linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
    linkJointTypes=jointTypes, linkJointAxis=axis
)


# print("Number of joints: ", p.getNumJoints(base_id))
# for j in range(p.getNumJoints(base_id)):
#     print(p.getJointInfo(base_id, j))


# for j in range(-1, p.getNumJoints(base_id)):
#     p.changeDynamics(base_id, j, jointLowerLimit=-0.5, jointUpperLimit=0.5)

p.changeDynamics(base_id, -1, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(base_id, 0, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(base_id, 1, jointLowerLimit=-0.5, jointUpperLimit=0.5)
p.changeDynamics(base_id, 2, jointLowerLimit=-0.3, jointUpperLimit=0.3)
p.changeDynamics(base_id, 3, jointLowerLimit=-0.3, jointUpperLimit=0.3)

joint_list = indices
p.setJointMotorControlArray(base_id, joint_list, p.VELOCITY_CONTROL, targetVelocities=len(joint_list) * [0],
                            forces=len(joint_list) * [1e-1])  # make plant responsive to external force


# p.changeDynamics(base_id, -1, jointDamping=10.0, maxJointVelocity=10.0, jointLowerLimit=-0.5, jointUpperLimit=0.5)

# p.setRealTimeSimulation(1)

# print("Number of joints: ", p.getNumJoints(base_id))

# for j in range(p.getNumJoints(base_id)):
#     # print(p.getJointInfo(base_id, j))
#     parent_idx = p.getJointInfo(base_id, j)[-1]
#
#     p.changeDynamics(base_id, parent_idx, jointLowerLimit=-1.57, jointUpperLimit=1.57)
#     # print(p.getJointInfo(base_id, j))
#     # input("")

# p.changeDynamics(base_id, 0, jointLowerLimit=-0.5, jointUpperLimit=0.5, jointLimitForce=0.0)
# p.changeDynamics(base_id, 1, jointLowerLimit=-0.5, jointUpperLimit=0.5, jointLimitForce=0.0)
# p.changeDynamics(base_id, -1, jointLowerLimit=-0.5, jointUpperLimit=0.5)
# p.changeDynamics(base_id, 1, jointLowerLimit=-0.5, jointUpperLimit=0.5)
# p.changeDynamics(base_id, 0, jointLowerLimit=-0.5, jointUpperLimit=0.5)
# p.changeDynamics(base_id, 2, jointLowerLimit=-0.5, jointUpperLimit=0.5)

# input("")

time_limit = 10
time_elapsed = time.time()

# log_id = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4,
#                 fileName="../../simulation_recordings/misc/plant_gone_crazy1.mp4")

p.setGravity(0, 0, -10)

while(1):


    # if(time.time() - time_elapsed > time_limit):
    #     break

    kp = 2000

    # for joint_idx in range(0, len(joint_list), 2):
    #
    #     plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = p.getJointState(base_id, joint_idx)
    #     plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = p.getJointState(base_id, joint_idx + 1)
    #
    #     p.applyExternalTorque(base_id, linkIndex=joint_idx + 1,
    #                           torqueObj=[-kp * plant_rot_joint_displacement_x, -kp * plant_rot_joint_displacement_y, 0],
    #                           flags=p.WORLD_FRAME)
    #
    #     kp = kp - 200

    plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = p.getJointState(base_id, 0)
    plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = p.getJointState(base_id, 0 + 1)

    p.applyExternalTorque(base_id, linkIndex=0 + 1,
                          torqueObj=[-kp * plant_rot_joint_displacement_x, -kp * plant_rot_joint_displacement_y, 0],
                          flags=p.WORLD_FRAME)

    p.stepSimulation()
    time.sleep(1/240.0)

# p.stopStateLogging(log_id)