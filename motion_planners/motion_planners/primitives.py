import time
from itertools import takewhile

import numpy as np
import pybullet

from .rrt import TreeNode, TreeNode_v2
from .utils import argmin, negate
import pybullet_tools.utils as pyb_tools_utils

import scripts.utils as s_utils

ASYMETRIC = True


def asymmetric_extend(q1, q2, extend_fn, backward=False):
    if backward and ASYMETRIC:
        return reversed(list(extend_fn(q2, q1))) # Forward model
    return extend_fn(q1, q2)


def extend_towards(tree, target, distance_fn, extend_fn, collision_fn, swap=False, tree_frequency=1):
    assert tree_frequency >= 1
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    safe = list(takewhile(negate(collision_fn), extend))

    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode(q, parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success


def extend_towards_with_angle_constraint_v2(tree, target, distance_fn, extend_fn, collision_fn, movable, robot, swap=False, tree_frequency=1):
    assert tree_frequency >= 1

    # print("Target: ",target, "Target configuration displayed...")
    # joints = pyb_tools_utils.get_movable_joints(robot)
    # pyb_tools_utils.set_joint_positions(robot, joints, target)
    # input("press enter to continue...")


    # Find the nearest node and restore its simulation state
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    last.restore_state()

    extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    # safe = list(takewhile(negate(collision_fn), extend))

    s_utils.step_sim()

    safe = []
    node_state = []
    for node in extend:

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        # print("+++++++++++++++++++++++++++++++++++++++++++++++++")
        # print("is_collision?", collision_fn(node))

        if not collision_fn(node):
            # print("--------- safe --------------")
            node_state.append(pyb_tools_utils.save_state())
            safe.append(node)
        else:
            # print("--------- collision --------------")
            break

        # for e,b in enumerate(movable):
        #     b.observe
        #     print(f"plant {e} deflection: {b.deflection}")

        # print("+++++++++++++++++++++++++++++++++++++++++++++++++")
        # input("")

    for i, q in enumerate(safe):
        if (i % tree_frequency == 0) or (i == len(safe) - 1):
            last = TreeNode_v2(q, state_id=node_state[i], parent=last)
            tree.append(last)
    success = len(extend) == len(safe)
    return last, success

def extend_towards_with_angle_constraint_v3(tree, target, distance_fn, extend_fn, collision_fn, movable, robot,
                                            joints, swap=False, tree_frequency=1):
    assert tree_frequency >= 1

    print("Moving towards new node...")

    # Find the nearest node and restore its simulation state
    last = argmin(lambda n: distance_fn(n.config, target), tree)
    last.restore_state()

    # extend = list(asymmetric_extend(last.config, target, extend_fn, backward=swap))
    # safe = list(takewhile(negate(collision_fn), extend))

    # step_sim()

    # safe = []
    # node_state = []
    # for node in extend:
    #
    #     # pyb_tools_utils.step_simulation()
    #     step_sim()
    #
    #     # print("+++++++++++++++++++++++++++++++++++++++++++++++++")
    #     # print("is_collision?", collision_fn(node))
    #
    #     if not collision_fn(node):
    #         # print("--------- safe --------------")
    #         node_state.append(pyb_tools_utils.save_state())
    #         safe.append(node)
    #     else:
    #         # print("--------- collision --------------")
    #         break
    #
    #     # for e,b in enumerate(movable):
    #     #     b.observe
    #     #     print(f"plant {e} deflection: {b.deflection}")
    #
    #     # print("+++++++++++++++++++++++++++++++++++++++++++++++++")
    #     # input("")

    pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, target)

    success = False

    i = 0
    while(True):

        s_utils.step_sim()
        ## TODO: check playground, smoothing
        current_joint_state = np.array(pybullet.getJointStates(robot, joints))[:,0]
        if(collision_fn(current_joint_state) and i > 0):

            last = TreeNode_v2(previous_joint_state, state_id=previous_state_id, parent=last)
            tree.append(last)

            break
        ## TODO: check for bugs in arriving at a particular joint state
        elif(np.linalg.norm(current_joint_state - target) < 0.80):

            last = TreeNode_v2(target, state_id=pyb_tools_utils.save_state(), parent=last)
            tree.append(last)
            success = True
            break

        i = i + 1
        previous_joint_state = current_joint_state
        previous_state_id = pyb_tools_utils.save_state()

        input("Press enter to step...")
        time.sleep(0.1)

    # for i, q in enumerate(safe):
    #     if (i % tree_frequency == 0) or (i == len(safe) - 1):
    #         last = TreeNode_v2(q, state_id=node_state[i], parent=last)
    #         tree.append(last)
    # success = len(extend) == len(safe)
    return last, success
