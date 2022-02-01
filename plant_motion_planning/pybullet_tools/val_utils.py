from .utils import get_movable_joints, get_pose, set_pose, set_joint_positions, set_joint_states

import numpy as np

# Params for joint configuration order of Val
ARM_DOF = 9
TORSO_DOF = 2
TORSO_START_INDEX = 4
LEFT_ARM_START_INDEX = 6
RIGHT_ARM_START_INDEX = 15

def get_arm_joints(body, is_left, include_torso=False):
    joints = get_movable_joints(body)
    arm_joints = []
    start_idx = LEFT_ARM_START_INDEX if is_left else RIGHT_ARM_START_INDEX

    if include_torso:
        arm_joints = get_torso_joints(body)

    for i in range(start_idx, start_idx+ARM_DOF):
        arm_joints.append(joints[i])

    return arm_joints

def get_torso_joints(body):
    joints = get_movable_joints(body)
    arm_joints = []
    for i in range(TORSO_START_INDEX, TORSO_START_INDEX + TORSO_DOF):
        arm_joints.append(joints[i])
    return arm_joints