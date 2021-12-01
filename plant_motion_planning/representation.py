import pybullet as p
import math

import numpy as np


class PlantJointState:
    pass


# Defining the raised height of the plant. Check plant_multi_dof_model.urdf for this value
RAISED_HEIGHT = 0.25



# simple 2 angle representation of a plant joint that can rotate freely in 2D
class TwoAngleRepresentation(PlantJointState):

    def __init__(self, base_id, link_id, joint_id = 0, deflection=0, orientation=0):
        self.bid = base_id
        self.lid = link_id
        self.jid = joint_id

        self.deflection = deflection
        self.orientation = orientation

    def observe(self):
        """Observe the deflection and orientation of the plant in the current environment"""

        # Get the link state from body ID and link ID
        link_state = p.getLinkState(self.bid, self.lid)

        link_pos = link_state[0]

        base_state = p.getBasePositionAndOrientation(self.bid)
        base_pos = base_state[0]

        diff_pos = np.array(link_pos) - np.array(base_pos)
        x, y, z = diff_pos[0], diff_pos[1], diff_pos[2] - RAISED_HEIGHT

        if abs(x) < 1e-5:
            x = 0.0
        if abs(y) < 1e-5:
            y = 0.0

        r = math.sqrt(x**2 + y**2 + z**2)

        # print(f"x: {x}, y: {y}, z: {z}, r: {r}")

        self.deflection = math.acos(z / r)
        self.orientation = math.atan2(y, x)

    def get_joint_id(self):
        return self.jid

    def set_state(self, deflection, orientation):
        """Set the state of the plant joint in the current environment"""
        # TODO
        self.deflection = deflection
        self.orientation = orientation


# simple 2 angle representation of a plant joint that can rotate freely in 2D
class TwoAngleRepresentation_mod(PlantJointState):

    def __init__(self, base_id, link_id, base_point, joint_id = 0, deflection=0, orientation=0):
        self.bid = base_id
        self.lid = link_id
        self.jid = joint_id

        self.deflection = deflection
        self.orientation = orientation

        self.base_point = np.array(base_point)
        # self.base_point[2] = 2 * self.base_point[2]
        init_pos = np.array(p.getLinkState(self.bid, self.lid)[0])
        self.init_vec = init_pos - base_point

        # print("self.base_point: ", self.base_point)
        # print("initial link pos: ", init_pos)
        # print("init vec: ", self.init_vec)
        # input("")

    def observe(self):
        """Observe the deflection and orientation of the plant in the current environment"""

        # Get the link state from body ID and link ID
        link_state = p.getLinkState(self.bid, self.lid)

        link_pos = link_state[0]
        link_ori_quat = link_state[1]
        # link_ori_rpy = p.getEulerFromQuaternion(link_ori_quat)
        #
        # print("Link ori rpy: ", np.sqrt((link_ori_rpy[0]**2) + (link_ori_rpy[1]**2)))


        cur_vec = np.array(link_pos) - np.array(self.base_point)

        # print("current link pos: ", link_pos, end="|")
        # print("base point: ", self.base_point, end="|")
        # print("current vector: ", cur_vec, end="|")

        # print("base point: ", self.base_point)
        # print("link pos: ", link_pos)
        # print("current vector: ", cur_vec)

        cos_angle = np.dot(self.init_vec, cur_vec) / (np.linalg.norm(self.init_vec) *
                                                                      np.linalg.norm(cur_vec))


        # print("init vec: ", self.init_vec)
        # print("cur_vec: ", cur_vec)
        # print("dot product: ", np.dot(self.init_vec, cur_vec))
        #
        # print("chk pt: ", np.dot(self.init_vec, cur_vec) / (np.linalg.norm(self.init_vec) *
        #                                                               np.linalg.norm(cur_vec)))
        # input("")
        if(cos_angle > 1):
            cos_angle = 1.0
        elif(cos_angle < -1):
            cos_angle = -1.0

        self.deflection = math.acos(cos_angle)

        # print(self.deflection)

    def get_joint_id(self):
        return self.jid

    def set_state(self, deflection, orientation):
        """Set the state of the plant joint in the current environment"""
        # TODO
        self.deflection = deflection
        self.orientation = orientation

# simple 2 angle representation of a plant joint that can rotate freely in 2D
class TwoAngleRepresentation_mod2(PlantJointState):

    def __init__(self, base_id, link_id, base_point, joint_id = 0, deflection=0, orientation=0):
        self.bid = base_id
        self.lid = link_id
        self.jid = joint_id

        self.deflection = deflection
        self.orientation = orientation

        self.base_point = np.array(p.getLinkState(self.bid, self.lid - 1)[0])
        init_pos = np.array(p.getLinkState(self.bid, self.lid)[0])
        self.init_vec = init_pos - self.base_point

        # print("self.base_point: ", self.base_point)
        # print("initial link pos: ", init_pos)
        # input("")

    def observe(self):
        """Observe the deflection and orientation of the plant in the current environment"""

        # Get the link state from body ID and link ID
        link_state = p.getLinkState(self.bid, self.lid)

        link_pos = link_state[0]

        base_point = np.array(p.getLinkState(self.bid, self.lid - 1)[0])
        cur_vec = np.array(link_pos) - base_point

        # print("current link pos: ", link_pos)
        # print("base point: ", self.base_point)
        # print(cur_vec)

        print("init vec: ", self.init_vec)
        print("cur_vec: ", cur_vec)
        print("dot product: ", np.dot(self.init_vec, cur_vec))

        print("chk pt: ", np.dot(self.init_vec, cur_vec) / (np.linalg.norm(self.init_vec) *
                                                                      np.linalg.norm(cur_vec)))
        input("")
        self.deflection = math.acos(np.dot(self.init_vec, cur_vec) / (np.linalg.norm(self.init_vec) *
                                                                      np.linalg.norm(cur_vec)))

    def get_joint_id(self):
        return self.jid

    def set_state(self, deflection, orientation):
        """Set the state of the plant joint in the current environment"""
        # TODO
        self.deflection = deflection
        self.orientation = orientation



class TwoAngleRepresentation_mod3(PlantJointState):

    def __init__(self, base_id, link_id, base_point, parent_lid, joint_id = 0, deflection=0, orientation=0):
        self.bid = base_id
        self.lid = link_id
        self.jid = joint_id

        self.deflection = deflection
        self.orientation = orientation

        self.base_point = np.array(base_point)
        # self.base_point[2] = 2 * self.base_point[2]
        init_pos = np.array(p.getLinkState(self.bid, self.lid)[0])
        self.init_vec = init_pos - base_point

        if(parent_lid == -1):
            self.parent_lid = None
            self.init_ori = p.getLinkState(self.bid, self.lid)[1]
        else:
            self.parent_lid = parent_lid
            self.init_ori = p.getDifferenceQuaternion(p.getLinkState(self.bid, self.lid)[1], p.getLinkState(self.bid,
                                                                                                        parent_lid)[1])


        # print("self.base_point: ", self.base_point)
        # print("initial link pos: ", init_pos)
        # print("init vec: ", self.init_vec)
        # input("")

    def observe(self):
        """Observe the deflection and orientation of the plant in the current environment"""

        # Get the link state from body ID and link ID
        link_state = p.getLinkState(self.bid, self.lid)

        link_pos = link_state[0]
        if(self.parent_lid is None):
            link_ori_quat = link_state[1]
        else:
            link_ori_quat = p.getDifferenceQuaternion(link_state[1], p.getLinkState(self.bid, self.parent_lid)[1])

        diff_quat = p.getDifferenceQuaternion(link_ori_quat, self.init_ori)
        diff_rpy = p.getEulerFromQuaternion(diff_quat)

        self.deflection = np.sqrt((diff_rpy[0]**2) + (diff_rpy[1]**2))

    def get_joint_id(self):
        return self.jid

    def set_state(self, deflection, orientation):
        """Set the state of the plant joint in the current environment"""
        # TODO
        self.deflection = deflection
        self.orientation = orientation



class CharacterizePlant2:

    def __init__(self, base_id, base_points, main_stem_indices, link_parent_indices, deflection_limit=-1):

        self.bid = base_id
        self.link_reps = []

        self.deflection_limit = deflection_limit

        self.base_points = base_points
        self.main_stem_indices = main_stem_indices

        for e, link_idx in enumerate(range(1, p.getNumJoints(base_id), 2)):
            self.link_reps.append(TwoAngleRepresentation_mod3(self.bid, link_idx, base_points[link_idx],
                                                              parent_lid=link_parent_indices[link_idx]))

        # input("both representations ready")

        self.deflections = []

    def observe_all(self):

        last_angle = 0
        counter = 0

        self.deflections.clear()

        for e, b in enumerate(self.link_reps):

            b.observe()
            # if(b.deflection > self.deflection_limit):
            #     return False

            # print(b.deflection)
            # print(last_angle)
            # print("======================")
            b.deflection = b.deflection - last_angle

            if(b.deflection < 0):
                b.deflection = 0

            self.deflections.append(b.deflection)

            if((2 * e + 2) in self.main_stem_indices):
                # if(True):
                # counter = counter + 1
                last_angle = b.deflection

        # print()
        # print("===========================")

    # def observe_all_and_check(self, deflection_limit):
    #
    #     last_angle = 0
    #     counter = 0
    #
    #     for e, b in enumerate(self.link_reps):
    #
    #         b.observe()
    #
    #         b.deflection = b.deflection - last_angle
    #
    #         if (b.deflection < 0):
    #             b.deflection = 0
    #
    #         if(b.deflection > deflection_limit):
    #             return True
    #
    #         if ((2 * e + 2) in self.main_stem_indices):
    #             # if(True):
    #             # counter = counter + 1
    #             last_angle = b.deflection
    #
    #     return False

    def observe_all_and_check(self, deflection_limit):

        self.observe_all()

        for deflection in self.deflections:

            if(deflection > deflection_limit):
                return True

        return False


class CharacterizePlant:

    def __init__(self, base_id, base_points, main_stem_indices, deflection_limit=-1):

        self.bid = base_id
        self.link_reps = []

        self.deflection_limit = deflection_limit

        self.base_points = base_points
        self.main_stem_indices = main_stem_indices

        for link_idx in range(1, p.getNumJoints(base_id), 2):
            self.link_reps.append(TwoAngleRepresentation_mod(self.bid, link_idx, base_points[link_idx]))

        # input("both representations ready")

        self.deflections = []

    def observe_all(self):

        last_angle = 0
        counter = 0

        self.deflections.clear()

        for e, b in enumerate(self.link_reps):

            b.observe()
            # if(b.deflection > self.deflection_limit):
            #     return False

            # print(b.deflection)
            # print(last_angle)
            # print("======================")
            b.deflection = b.deflection - last_angle

            if(b.deflection < 0):
                b.deflection = 0

            self.deflections.append(b.deflection)

            if((2 * e + 2) in self.main_stem_indices):
            # if(True):
                # counter = counter + 1
                last_angle = b.deflection

        # print()
        # print("===========================")

    # def observe_all_and_check(self, deflection_limit):
    #
    #     last_angle = 0
    #     counter = 0
    #
    #     for e, b in enumerate(self.link_reps):
    #
    #         b.observe()
    #
    #         b.deflection = b.deflection - last_angle
    #
    #         if (b.deflection < 0):
    #             b.deflection = 0
    #
    #         if(b.deflection > deflection_limit):
    #             return True
    #
    #         if ((2 * e + 2) in self.main_stem_indices):
    #             # if(True):
    #             # counter = counter + 1
    #             last_angle = b.deflection
    #
    #     return False

    def observe_all_and_check(self, deflection_limit):

        self.observe_all()

        for deflection in self.deflections:

            if(deflection > deflection_limit):
                return True

        return False
