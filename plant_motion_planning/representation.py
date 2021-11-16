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
        # input("")

    def observe(self):
        """Observe the deflection and orientation of the plant in the current environment"""

        # Get the link state from body ID and link ID
        link_state = p.getLinkState(self.bid, self.lid)

        link_pos = link_state[0]

        cur_vec = np.array(link_pos) - np.array(self.base_point)

        # print("current link pos: ", link_pos)
        # print("base point: ", self.base_point)
        # print(cur_vec)

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


class CharacterizePlant:

    def __init__(self, base_id, base_points, main_stem_indices, deflection_limit=-1):

        self.bid = base_id
        self.link_reps = []

        self.deflection_limit = deflection_limit

        self.base_points = base_points
        self.main_stem_indices = main_stem_indices

        for link_idx in range(1, p.getNumJoints(base_id), 2):
            self.link_reps.append(TwoAngleRepresentation_mod(self.bid, link_idx, base_points[link_idx]))



    def observe_all(self):

        last_angle = 0
        counter = 0

        print("===========================")
        for e, b in enumerate(self.link_reps):

            b.observe()
            # if(b.deflection > self.deflection_limit):
            #     return False

            b.deflection = b.deflection - last_angle

            if(b.deflection < 0):
                b.deflection = 0

            print("Deflection of link %d: %f" % (e, b.deflection))

            if((2 * e + 2) in self.main_stem_indices):
            # if(True):
                # counter = counter + 1
                last_angle = b.deflection

        print("===========================")

        # return True
