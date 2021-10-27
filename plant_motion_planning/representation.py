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
