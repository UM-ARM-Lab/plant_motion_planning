import time

import pybullet
import pybullet as p

# from pybullet_tools.utils import set_joint_positions, get_movable_joints
import pybullet_tools.utils as pyb_utils
from .meta import random_restarts_v4
from .primitives import extend_towards, extend_towards_with_angle_constraint_v2, \
    extend_towards_with_angle_constraint_v3
from .rrt import TreeNode, configs, TreeNode_v2
from .utils import irange, RRT_ITERATIONS, INF, elapsed_time
from pybullet_tools import utils as pyb_tools_utils

import scripts.utils as s_utils
from random import random

def wrap_collision_fn(collision_fn):
    # TODO: joint limits
    # import inspect
    # print(inspect.getargspec(collision_fn))
    # print(dir(collision_fn))
    def fn(q1, q2):
        try:
            return collision_fn(q1, q2)
        except TypeError:
            return collision_fn(q2)
    return fn


def rrt(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: goal sampling function connected to a None node
    start_time = time.time()

    goal_probability = 0.5

    # if collision_fn(start) or collision_fn(goal):
    #     return None

    if collision_fn(start):
        return None

    # if not callable(goal):
    #     g = goal
    #     goal = lambda: g

    # start_state_id = p.saveState()

    # joints = pyb_tools_utils.get_movable_joints(robot)
    # pyb_tools_utils.set_joint_positions(robot, joints, goal)
    # goal_state_id = p.saveState()

    input("")

    p.restoreState(start_state_id)


    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode_v2(start, state_id=start_state_id)], [TreeNode_v2(goal)] # TODO: allow a tree to be prespecified (possibly as start)
    tree1 = nodes1

    # Save the state of the environment at the start node
    # nodes1.save_state()

    max_iterations = 50

    for iteration in irange(max_iterations):

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        if elapsed_time(start_time) >= max_time:
            print("=========================================")
            print("Time limit exceeded!")
            print("=========================================")
            break

        # swap = len(nodes1) > len(nodes2)
        # tree1, tree2 = nodes1, nodes2
        # if swap:
        #     tree1, tree2 = nodes2, nodes1

        # p.stepSimulation(physicsClientId=CLIENT)
        # step_simulation()

        # pyb_tools_utils.step_simulation()

        if random() <= goal_probability or iteration == 0:
            target = goal
        else:
            target = sample_fn()

        last1, success = extend_towards_with_angle_constraint_v2(tree1, target, distance_fn, extend_fn, collision_fn, movable,
                                                           robot, False, **kwargs)
        # last2, success = extend_towards_with_angle_constraint_v2(tree2, last1.config, distance_fn, extend_fn,
        #                                                          collision_fn, movable, robot, not swap, **kwargs)

        if success and target == goal:
            # path1, path2 = last1.retrace(), last2.retrace()
            path1 = last1.retrace()
            # if swap:
            #     path1, path2 = path2, path1
            #print('{} max_iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            path = configs(path1)
            # TODO: return the trees
            return path
    return None

#################################################################


# temp
def check_forward_path2(robot, path2, collision_fn, nodes1, nodes2):

    last_tree1 = nodes1[-1]
    # tree2.append(0)
    nodes2.clear()

    last_tree1.restore_state()

    pyb_utils.set_joint_positions(robot, pyb_utils.get_movable_joints(robot), last_tree1.config)
    pybullet.setJointMotorControlArray(robot, pyb_utils.get_movable_joints(robot), pybullet.POSITION_CONTROL,
                                       last_tree1.config, positionGains=7 * [0.01])

    for t in range(10):
        s_utils.step_sim()

    for e, node in enumerate(path2[1:]):
        if(collision_fn(node.config)):
            print("*********************************************")
            print("Backward path not feasible!!")
            print("*********************************************")
            # input("Saving all next node states until goal! Press enter to continue...")

            # tree2.append(path2[-1])
            for i in range(e+1, len(path2)):
                nodes2.append(path2[i])

            # print("path2: ",  path2)
            # print("tree2: ", tree2)
            # exit()

            ## TODO: check this statement
            nodes1 = nodes1[:-2]

            return False
        else:
            # TODO: make sure all data related to this node is being updated!

            tree1_new_node = TreeNode_v2(node.config, parent=last_tree1)
            tree1_new_node.save_state()

            # node.parent = last_tree1
            # node.save_state()
            nodes1.append(tree1_new_node)
            last_tree1 = tree1_new_node


    return True

def check_forward_path(path2, collision_fn, tree1, tree2):

    last_tree1 = tree1[-1]
    # tree2.append(0)
    tree2 = []

    last_tree1.restore_state()

    for e, node in enumerate(path2[1:]):
        if(collision_fn(node.config)):
            print("*********************************************")
            print("Backward path not feasible!!")
            print("*********************************************")
            # input("Saving all next node states until goal! Press enter to continue...")

            tree2.append(path2[-1])
            # for i in range(e+1, len(path2)):
            #     tree2.append(path2[i])

            # print("path2: ",  path2)
            # print("tree2: ", tree2)
            # exit()

            ## TODO: check this statement
            tree1 = tree1[:-2]

            return False
        else:
            # TODO: make sure all data related to this node is being updated!

            tree1_new_node = TreeNode_v2(node.config, parent=last_tree1)
            tree1_new_node.save_state()

            # node.parent = last_tree1
            # node.save_state()
            tree1.append(tree1_new_node)
            last_tree1 = tree1_new_node


    return True


def rrt_connect_v5(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                   max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: goal sampling function connected to a None node

    joints = pyb_tools_utils.get_movable_joints(robot)
    pyb_tools_utils.set_joint_positions(robot, joints, start)

    # Wait for some time to let things settle down
    print("RRT: Waiting for the environment to settle...")
    t = 0
    while(t <= 100):
        t = t + 1
        s_utils.step_sim()
        time.sleep(0.01)

    start_time = time.time()
    # if collision_fn(start) or collision_fn(goal):
    #     return None

    if collision_fn(start):
        return None
    # start_state_id = p.saveState()

    pyb_tools_utils.set_joint_positions(robot, joints, goal)

    t = 0
    while(t <= 100):
        t = t + 1
        s_utils.step_sim()
        time.sleep(0.01)

    goal_state_id = p.saveState()

    # input("")

    p.restoreState(start_state_id)


    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode_v2(start, state_id=start_state_id)], [TreeNode_v2(goal, state_id=goal_state_id)] # TODO: allow a tree to be prespecified (possibly as start)

    # Save the state of the environment at the start node
    # nodes1.save_state()

    swap = True

    for iteration in irange(max_iterations):

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        if elapsed_time(start_time) >= max_time:
            break

        # swap = len(nodes1) > len(nodes2)
        swap = not swap
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        # p.stepSimulation(physicsClientId=CLIENT)
        # step_simulation()

        # pyb_tools_utils.step_simulation()

        target = sample_fn()

        last1, _ = extend_towards_with_angle_constraint_v2(tree1, target, distance_fn, extend_fn, collision_fn, movable,
                                                           robot, swap, **kwargs)
        last2, success = extend_towards_with_angle_constraint_v2(tree2, last1.config, distance_fn, extend_fn,
                                                                 collision_fn, movable, robot, not swap, **kwargs)

        if success:

            path1, path2 = last1.retrace(), last2.retrace()

            if swap:
                path1, path2 = path2, path1

            # path1 - path from source node to last target node
            # path2 - path from goal node to last target node

            # Reversing path2 so that it starts from last target node and goes all the way till the goal node
            path2 = path2[::-1]

            # input("press enter to check forward path...")
            # Checking forward path
            if(check_forward_path2(path2, collision_fn, nodes1, nodes2)):
                path = configs(path1[:-1] + path2)
                # TODO: return the trees
                node_path = path1[:-1] + path2
                return path, node_path
                # return path

            swap = True

            #print('{} max_iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            # path = configs(path1[:-1] + path2[::-1])
            # # TODO: return the trees
            # return path
    return None, None
    # return None

def rrt_connect_v4(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                   max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: goal sampling function connected to a None node

    joints = pyb_tools_utils.get_movable_joints(robot)
    pyb_tools_utils.set_joint_positions(robot, joints, start)
    pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, start, positionGains=7 * [0.01])

    # Wait for some time to let things settle down
    print("RRT: Waiting for the environment to settle...")
    t = 0
    while(t <= 100):
        t = t + 1
        s_utils.step_sim()
        # time.sleep(0.01)

    start_time = time.time()
    # if collision_fn(start) or collision_fn(goal):
    #     return None

    if collision_fn(start):
        return None
    # start_state_id = p.saveState()

    pyb_tools_utils.set_joint_positions(robot, joints, goal)
    pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, goal, positionGains=7 * [0.01])

    t = 0
    while(t <= 100):
        t = t + 1
        s_utils.step_sim()
        time.sleep(0.01)

    goal_state_id = p.saveState()

    # input("")

    p.restoreState(start_state_id)


    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode_v2(start, state_id=start_state_id)], [TreeNode_v2(goal, state_id=goal_state_id)] # TODO: allow a tree to be prespecified (possibly as start)

    # Save the state of the environment at the start node
    # nodes1.save_state()

    swap = True

    for iteration in irange(max_iterations):

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        if elapsed_time(start_time) >= max_time:
            break

        # swap = len(nodes1) > len(nodes2)
        swap = not swap
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        # p.stepSimulation(physicsClientId=CLIENT)
        # step_simulation()

        # pyb_tools_utils.step_simulation()

        target = sample_fn()

        last1, _ = extend_towards_with_angle_constraint_v2(tree1, target, distance_fn, extend_fn, collision_fn, movable,
                                                           robot, swap, **kwargs)
        last2, success = extend_towards_with_angle_constraint_v2(tree2, last1.config, distance_fn, extend_fn,
                                                                 collision_fn, movable, robot, not swap, **kwargs)

        if success:

            path1, path2 = last1.retrace(), last2.retrace()

            if swap:
                path1, path2 = path2, path1

            # path1 - path from source node to last target node
            # path2 - path from goal node to last target node

            # Reversing path2 so that it starts from last target node and goes all the way till the goal node
            path2 = path2[::-1]

            # input("press enter to check forward path...")
            # Checking forward path
            if(check_forward_path2(robot, path2, collision_fn, nodes1, nodes2)):
                path = configs(path1[:-1] + path2)
                # TODO: return the trees
                node_path = path1[:-1] + path2
                return path, node_path
                # return path

            swap = True

            #print('{} max_iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            # path = configs(path1[:-1] + path2[::-1])
            # # TODO: return the trees
            # return path
    return None, None
    # return None

def rrt_connect_v3(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                   max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: goal sampling function connected to a None node
    start_time = time.time()
    # if collision_fn(start) or collision_fn(goal):
    #     return None

    if collision_fn(start):
        return None
    # start_state_id = p.saveState()

    joints = pyb_tools_utils.get_movable_joints(robot)
    pyb_tools_utils.set_joint_positions(robot, joints, goal)

    t = 0
    while(t <= 100):
        t = t + 1
        s_utils.step_sim()
        time.sleep(0.01)

    goal_state_id = p.saveState()

    input("")

    p.restoreState(start_state_id)


    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode_v2(start, state_id=start_state_id)], [TreeNode_v2(goal, state_id=goal_state_id)] # TODO: allow a tree to be prespecified (possibly as start)

    # Save the state of the environment at the start node
    # nodes1.save_state()

    for iteration in irange(max_iterations):

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        if elapsed_time(start_time) >= max_time:
            break

        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        # p.stepSimulation(physicsClientId=CLIENT)
        # step_simulation()

        # pyb_tools_utils.step_simulation()

        target = sample_fn()

        last1, _ = extend_towards_with_angle_constraint_v3(tree1, target, distance_fn, extend_fn, collision_fn, movable,
                                                           robot, joints, swap, **kwargs)
        last2, success = extend_towards_with_angle_constraint_v3(tree2, last1.config, distance_fn, extend_fn,
                                                                 collision_fn, movable, robot, joints, not swap,
                                                                 **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            #print('{} max_iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            path = configs(path1[:-1] + path2[::-1])
            # TODO: return the trees
            return path
    return None

def rrt_connect_v2(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_iterations: Maximum number of iterations - int
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: goal sampling function connected to a None node
    start_time = time.time()
    # if collision_fn(start) or collision_fn(goal):
    #     return None

    if collision_fn(start):
        return None
    # start_state_id = p.saveState()

    joints = pyb_tools_utils.get_movable_joints(robot)
    pyb_tools_utils.set_joint_positions(robot, joints, goal)
    goal_state_id = p.saveState()

    input("")

    p.restoreState(start_state_id)


    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode_v2(start, state_id=start_state_id)], [TreeNode_v2(goal, state_id=goal_state_id)] # TODO: allow a tree to be prespecified (possibly as start)

    # Save the state of the environment at the start node
    # nodes1.save_state()

    for iteration in irange(max_iterations):

        # pyb_tools_utils.step_simulation()
        s_utils.step_sim()

        if elapsed_time(start_time) >= max_time:
            break

        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        # p.stepSimulation(physicsClientId=CLIENT)
        # step_simulation()

        # pyb_tools_utils.step_simulation()

        target = sample_fn()

        last1, _ = extend_towards_with_angle_constraint_v2(tree1, target, distance_fn, extend_fn, collision_fn, movable,
                                  robot, swap, **kwargs)
        last2, success = extend_towards_with_angle_constraint_v2(tree2, last1.config, distance_fn, extend_fn,
                                                                 collision_fn, movable, robot, not swap, **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            #print('{} max_iterations, {} nodes'.format(iteration, len(nodes1) + len(nodes2)))
            path = configs(path1[:-1] + path2[::-1])
            # TODO: return the trees
            return path
    return None

#################################################################


def birrt_v5(robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: deprecate
    from .meta import random_restarts_v2
    # path, node_path = rrt_connect_v4(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn,
    #                                  collision_fn, movable, max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs)
    # return path

    solutions = random_restarts_v4(rrt_connect_v4, robot, start_state_id, start, goal, distance_fn, sample_fn,
                                   extend_fn, collision_fn, movable, max_solutions=1, **kwargs)

    if not solutions:
        return None
    return solutions[0]

def birrt_v4(robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: deprecate
    from .meta import random_restarts_v2
    # path, node_path = rrt_connect_v4(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn,
    #                                  collision_fn, movable, max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs)
    # return path

    solutions = random_restarts_v4(rrt_connect_v4, robot, start_state_id, start, goal, distance_fn, sample_fn,
                                   extend_fn, collision_fn, movable, max_solutions=1, **kwargs)

    if not solutions:
        return None
    return solutions[0]


def birrt_v2(robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: deprecate
    from .meta import random_restarts_v2
    solutions = random_restarts_v2(rrt_connect_v2, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn,
                                   collision_fn, movable, max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]

def birrt_v3(robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param kwargs: Keyword arguments
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: deprecate
    from .meta import random_restarts_v2
    solutions = random_restarts_v2(rrt_connect_v3, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn,
                                   collision_fn, movable, max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]
