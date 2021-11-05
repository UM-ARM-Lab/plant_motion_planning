# from random import random

import random

import numpy as np
import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Remove debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

stem_base_spacing = 0.4

def create_stem_element(stem_half_length, stem_half_width, stem_half_height):

    col_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[stem_half_length, stem_half_width, stem_half_height],
        collisionFramePosition=[0, 0, stem_base_spacing + stem_half_height]
    )
    vis_stem_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[stem_half_length, stem_half_width, stem_half_height],
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

    base_half_width = 0.2
    base_half_length = 0.2
    base_half_height = 0.2

    col_base_id = p.createCollisionShape(
        p.GEOM_BOX, halfExtents=[base_half_width, base_half_length, base_half_height]
    )
    vis_base_id = -1

    # Base position and orientation
    base_pos = [0, 0, base_half_height]
    base_ori = [0, 0, 0, 1]

    # Base mass - Keep high to simulate a fixed base
    base_mass = 100000

    # Creating variables to keep track of the stem indices
    current_index = 0
    main_stem_index = 0
    indices = []

    ###############

    stem_half_length = 0.2
    stem_half_width = 0.2
    stem_half_height = 0.6

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
    num_branches_per_stem = 2
    branch_count = 0

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
                    branch_count = branch_count + 1
                    break


            if(np.abs(x) > np.abs(y)):
                if(x > 0):
                    pitch = np.random.uniform(low=0.5, high=1.5)
                else:
                    pitch = np.random.uniform(low=-1.5, high=-0.5)
                roll = 0.0
            else:
                if (y > 0):
                    roll = np.random.uniform(low=-1.5, high=-0.5)
                else:
                    roll = np.random.uniform(low=0.5, high=1.5)

                pitch = 0.0

            yaw = np.random.uniform(low=-0.5, high=0.5)

            # if(x < 0):
            #     pitch = np.random.uniform(low=-1.5, high=-0.5)
            # else:
            #     pitch = np.random.uniform(low=0.5, high=1.5)
            #
            # if(y < 0):
            #     roll = np.random.uniform(low=-1.5, high=-0.5)
            # else:
            #     roll = np.random.uniform(low=0.5, high=1.5)

            v1_pos = [x, y, stem_base_spacing + stem_half_height]

            print("roll: ", roll)
            print("pitch: ", pitch)

            v1_ori = p.getQuaternionFromEuler((roll, pitch, yaw))

            current_index = current_index + 2
            indices = indices + [main_stem_index, current_index - 1]

        else:
            exit_out = True
            break
            # # Stack this stem on top of the old stem vertically
            # vert_stem_count = vert_stem_count + 1
            # v1_pos = [0, 0, stem_base_spacing + (2 * stem_half_height)]
            # v1_ori = p.getQuaternionFromEuler((0, 0, 0.0))
            #
            # current_index = current_index + 2
            # indices = indices + [main_stem_index, current_index - 1]
            # main_stem_index = current_index


        link_mass = 1
        col_stem_id, vis_stem_id = create_stem_element(stem_half_length, stem_half_width, stem_half_height)

        link_Masses  = link_Masses + [link_mass, link_mass]
        linkCollisionShapeIndices = linkCollisionShapeIndices + col_stem_id
        linkVisualShapeIndices = linkVisualShapeIndices + vis_stem_id

        linkPositions = linkPositions + [v1_pos, [0, 0, 0]]
        linkOrientations = linkOrientations + [v1_ori, [0, 0, 0, 1]]

        linkInertialFramePositions = linkInertialFramePositions + [[0, 0, 0], [0, 0, 0]]
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

# Making the plant
base_id = p.createMultiBody(
    base_mass, col_base_id, vis_base_id, base_pos, base_ori,
    linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
    linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
    linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
    linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
    linkJointTypes=jointTypes, linkJointAxis=axis
)


p.setGravity(0, 0, -10)
p.setRealTimeSimulation(1)

while(1):
    time.sleep(1/240.0)