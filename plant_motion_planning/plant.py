import random
import numpy as np
import pybullet as p

class Plant:
    def __init__(self, num_branches_per_stem, total_num_vert_stems, total_num_extensions, base_pos_xy):
        self.base_id, self.joint_list = self.create_random_plant(num_branches_per_stem, total_num_vert_stems, total_num_extensions, base_pos_xy)
    
    def create_stem_element(self, stem_half_length, stem_half_width, current_index, scaling_factor, stem_half_height, stem_base_spacing):
        stem_half_height[current_index] = np.random.uniform(low=scaling_factor * 0.7, high=scaling_factor * 1.0)

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

    def intersection_with_others(self, x, y, history, tolerance=0.03):
        if(len(history) == 0):
            history.append((x,y))
            return False
        else:

            for prev in history:
                if(np.abs(x - prev[0]) < tolerance and np.abs(y - prev[1]) < tolerance):
                    return True

            history.append((x, y))
            return False

    def create_plant_params(self,
                            num_branches_per_stem,
                            total_num_vert_stems,
                            total_num_extensions, base_pos_xy,
                            base_mass = 100000 * 10000):

        # num_branches_per_stem = 2
        # total_num_vert_stems = 3
        # total_num_extensions = 1

        current_index = 0
        main_stem_index = 0

        scaling_factor = 0.25

        base_points = {}
        main_stem_indices = []
        link_parent_indices = {}

        stem_half_height = {}
        stem_base_spacing = 0

        stem_half_length = scaling_factor * 0.1
        stem_half_width = scaling_factor * 0.1

        base_half_width = np.random.uniform(low=scaling_factor * 0.07, high=scaling_factor * 0.15)
        base_half_length = np.random.uniform(low=scaling_factor * 0.07, high=scaling_factor * 0.15)
        base_half_height = np.random.uniform(low=scaling_factor * 0.07, high=scaling_factor * 0.15)
        # print("base height: ", 2 * base_half_height)


        col_base_id = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[base_half_width, base_half_length, base_half_height]
        )
        vis_base_id = -1

        # Base position and orientation
        base_pos = [base_pos_xy[0], base_pos_xy[1], base_half_height]
        base_ori = [0, 0, 0, 1]

        # Base mass - Keep high to simulate a fixed base
        # base_mass = 100000

        ###############

        # stem_half_length = 0.1
        # stem_half_width = 0.1
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

        # total_num_stems = 2
        history = []

        exit_out = False
        ith_stem = 0
        # num_branches_per_stem = 2
        branch_count = 1
        # total_num_vert_stems = 1
        # total_num_extensions = 1
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
                main_stem_indices.append(main_stem_index)

                base_points[current_index-1] = [base_pos_xy[0], base_pos_xy[1], 2 * v1_pos[2]]

                link_parent_indices[current_index-1] = -1


            elif(branch_count <= num_branches_per_stem):


                if(num_extensions <= total_num_extensions):
                    v1_pos = [0, 0, stem_base_spacing + (2 * stem_half_height[current_index])]

                    if(random.random() < 0.5):
                        roll = np.random.uniform(low=scaling_factor * -0.5,high=scaling_factor * 0.5)
                        pitch = 0.0
                    else:
                        pitch = np.random.uniform(low=scaling_factor * -0.5,high=scaling_factor * 0.5)
                        roll = 0.0

                    v1_ori = p.getQuaternionFromEuler((roll, pitch, 0.0))

                    indices = indices + [current_index, current_index + 1]
                    current_index = current_index + 2

                    link_parent_indices[current_index-1] = current_index - 2 - 1

                    if(num_extensions == total_num_extensions):
                        branch_count = branch_count + 1

                    num_extensions = num_extensions + 1



                else:
                    num_extensions = 1

                    while(1):
                        if(random.random() < 0.5):
                            x = random.choice([scaling_factor * 0.2,scaling_factor * -0.2])
                            y = np.random.uniform(low=0.0, high=scaling_factor * 0.2)
                        else:
                            y = random.choice([scaling_factor * 0.2,scaling_factor * -0.2])
                            x = np.random.uniform(low=0.0, high=scaling_factor * 0.2)

                        if(self.intersection_with_others(x,y, history, tolerance=scaling_factor * 0.2)):
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

                    link_parent_indices[current_index-1] = main_stem_index - 1

                base_points[current_index-1] = [base_pos_xy[0], base_pos_xy[1], v1_pos[2]]


            elif(vert_stem_count == total_num_vert_stems):
                exit_out = True
                break
            else:
                # Stack this stem on top of the old stem vertically
                vert_stem_count = vert_stem_count + 1
                branch_count = 1
                history = []

                v1_pos = [0, 0, stem_base_spacing + (2 * stem_half_height[main_stem_index])]
                # v1_pos = [base_pos_xy[0], base_pos_xy[1], stem_base_spacing + (2 * stem_half_height[main_stem_index])]

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
                link_parent_indices[current_index-1] = main_stem_index - 1

                main_stem_index = current_index
                main_stem_indices.append(main_stem_index)

                base_points[current_index-1] = [base_pos_xy[0], base_pos_xy[1], v1_pos[2]]


            link_mass = [0, 10]
            col_stem_id, vis_stem_id = self.create_stem_element(stem_half_length, stem_half_width, current_index, scaling_factor, stem_half_height, stem_base_spacing)

            # link_Masses  = link_Masses + [link_mass, link_mass]
            link_Masses  = link_Masses + link_mass
            linkCollisionShapeIndices = linkCollisionShapeIndices + col_stem_id
            linkVisualShapeIndices = linkVisualShapeIndices + vis_stem_id

            linkPositions = linkPositions + [v1_pos, [0, 0, 0]]
            linkOrientations = linkOrientations + [v1_ori, [0, 0, 0, 1]]

            linkInertialFramePositions = linkInertialFramePositions + [[0, 0, 0], [0, 0, scaling_factor * 1]]
            linkInertialFrameOrientations = linkInertialFrameOrientations + [[0, 0, 0, 1], [0, 0, 0, 1]]

            jointTypes = jointTypes + [p.JOINT_REVOLUTE, p.JOINT_REVOLUTE]
            axis = axis + [[1, 0, 0], [0, 1, 0]]


        return [base_mass, col_base_id, vis_base_id, base_pos, base_ori], [link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions,
                            linkOrientations, linkInertialFramePositions, linkInertialFrameOrientations, indices,
                            jointTypes, axis], main_stem_indices


    def create_random_plant(self, num_branches_per_stem, total_num_vert_stems, total_num_extensions, base_pos_xy, stem_half_length=0.1,
                            stem_half_width=0.1):
        self.base_params, self.stems_params, self.main_stem_indices = self.create_plant_params(num_branches_per_stem, total_num_vert_stems,
                                                                        total_num_extensions, base_pos_xy)
        return self.restore_plant()
    
    def restore_plant(self):
        base_mass, col_base_id, vis_base_id, base_pos, base_ori = self.base_params

        link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
        linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = self.stems_params

        base_id = p.createMultiBody(
            base_mass, col_base_id, vis_base_id, base_pos, base_ori,
            linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
            linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
            linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
            linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
            linkJointTypes=jointTypes, linkJointAxis=axis
        )
        joint_list = indices

        for j in range(-1, p.getNumJoints(base_id)):
            p.changeDynamics(base_id, j, jointLowerLimit=-1.5, jointUpperLimit=1.5, jointDamping=10, linearDamping=1.5)

        return base_id, joint_list