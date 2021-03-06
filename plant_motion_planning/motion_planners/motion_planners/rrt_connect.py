import time

from .meta import random_restarts_with_controls, random_restarts_single_plant, random_restarts_multiworld_benchmark
from .primitives import extend_towards, extend_towards_with_controls, extend_towards_fn_single_plant, \
    extend_towards_multiworld_benchmark
from .rrt import TreeNode, configs
from .utils import irange, RRT_ITERATIONS, INF, elapsed_time

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



def rrt_connect_single_plant(robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                              max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    Function to perform Bi-RRT from start to goal for a single stemmed plant
    """
    start_time = time.time()

    # Initialize forward and backward trees
    nodes1, nodes2 = [TreeNode(start)], [TreeNode(goal)] 
    
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        # Sample a new node
        target = sample_fn()

        # Extend toward newly sampled node
        last1, _ = extend_towards_fn_single_plant(tree1, joints, target, distance_fn, extend_fn, collision_fn, robot,
                                                swap, **kwargs)

        # Extend toward last node created in previous extension operation
        last2, success = extend_towards_fn_single_plant(tree2, joints, last1.config, distance_fn, extend_fn, collision_fn,
                                                      robot, not swap, **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            path = configs(path1[:-1] + path2[::-1])
            return path

    return None


def rrt_connect_multiworld_benchmark(multi_world_env, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                              max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    Path computation using the Bi-RRT algorithm
    """
    start_time = time.time()

    nodes1, nodes2 = [TreeNode(start)], [TreeNode(goal)] 

    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        target = sample_fn()
        last1, _ = extend_towards_multiworld_benchmark(tree1, target, distance_fn, extend_fn, collision_fn, multi_world_env,
                                                swap, **kwargs)
        last2, success = extend_towards_multiworld_benchmark(tree2, last1.config, distance_fn, extend_fn, collision_fn,
                                                      multi_world_env, not swap, **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            path = configs(path1[:-1] + path2[::-1])
            return path
    return None

def rrt_connect_with_controls(robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                max_iterations=RRT_ITERATIONS, max_time=INF, **kwargs):
    """
    This function performs Bi-RRT computation from start to goal and returns a path if possible.
    """
    start_time = time.time()

    # Create two trees, a forward and backward tree
    nodes1, nodes2 = [TreeNode(start)], [TreeNode(goal)] 
    
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break

        # Swap flag
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        target = sample_fn()

        # Extend toward newly sampled node 
        last1, _ = extend_towards_with_controls(tree1, joints, target, distance_fn, extend_fn, collision_fn, robot,
                                  swap, **kwargs)

        # Extend toward last node in the extension operation performed by other tree
        last2, success = extend_towards_with_controls(tree2, joints, last1.config, distance_fn, extend_fn, collision_fn,
                                        robot, not swap, **kwargs)

        if success:
            path1, path2 = last1.retrace(), last2.retrace()
            if swap:
                path1, path2 = path2, path1
            path = configs(path1[:-1] + path2[::-1])
            return path
    
    return None


def rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
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
    if collision_fn(start) or collision_fn(goal):
        return None
    # TODO: support continuous collision_fn with two arguments
    #collision_fn = wrap_collision_fn(collision_fn)
    nodes1, nodes2 = [TreeNode(start)], [TreeNode(goal)] # TODO: allow a tree to be prespecified (possibly as start)
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        swap = len(nodes1) > len(nodes2)
        tree1, tree2 = nodes1, nodes2
        if swap:
            tree1, tree2 = nodes2, nodes1

        target = sample_fn()
        last1, _ = extend_towards(tree1, target, distance_fn, extend_fn, collision_fn,
                                  swap, **kwargs)
        # input("moved from tree1...")

        last2, success = extend_towards(tree2, last1.config, distance_fn, extend_fn, collision_fn,
                                        not swap, **kwargs)
        # input("moved from tree2...")

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

def birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
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
    from .meta import random_restarts
    solutions = random_restarts(rrt_connect, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]


def birrt_multiworld_benchmark(multi_world_env, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    """
    Bi-RRT computation and smoothing
    """
    solutions = random_restarts_multiworld_benchmark(rrt_connect_multiworld_benchmark, multi_world_env, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                              max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]

def birrt_with_controls(robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
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

    # Calling a function to perform Bi-RRT planning and then smoothing.
    solutions = random_restarts_with_controls(rrt_connect_with_controls, robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]

def birrt_single_plant(robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
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

    # Planning and smoothing
    solutions = random_restarts_single_plant(rrt_connect_single_plant, robot, joints, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                              max_solutions=1, **kwargs)
    if not solutions:
        return None
    return solutions[0]
