import time

import pybullet

import pybullet_tools.utils
# from scripts.utils import step_sim
import scripts.utils as s_utils
from .lattice import lattice
from .lazy_prm import lazy_prm
from .prm import prm
from .rrt import rrt
# from .rrt_connect import rrt_connect, birrt
from .rrt_star import rrt_star
from .utils import INF

from .smoothing import smooth_path, smooth_path_v2, smooth_path_v4, smooth_path_with_controls
from .utils import RRT_RESTARTS, RRT_SMOOTHING, INF, irange, elapsed_time, compute_path_cost, default_selector


def direct_path(start, goal, extend_fn, collision_fn):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: version which checks whether the segment is valid
    if collision_fn(start) or collision_fn(goal):
        # TODO: return False
        return None
    path = list(extend_fn(start, goal))
    path = [start] + path
    if any(collision_fn(q) for q in default_selector(path)):
        return None
    return path
    # path = [start]
    # for q in extend_fn(start, goal):
    #     if collision_fn(q):
    #         return None
    #     path.append(q)
    # return path

def check_direct(start, goal, extend_fn, collision_fn):
    if any(collision_fn(q) for q in [start, goal]):
        return False
    return direct_path(start, goal, extend_fn, collision_fn)

#################################################################


def random_restarts_v4(solve_fn, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                       movable, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                       success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Paths [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    solutions = []
    # path = check_direct(start, goal, extend_fn, collision_fn)
    # if path is False:
    #     return None
    # if path is not None:
    #     solutions.append(path)

    smooth_attempts = 5

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path_data = solve_fn(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                        max_time=attempt_time, **kwargs)

        if path_data is None or path_data[0] is None:
            continue

        path, node_path = path_data

        # else:
        #     path, node_path = path_data

        # input("Path found, press enter to start smoothing...")

        for smooth_attempt in range(smooth_attempts):
            path_smoothed = smooth_path_v4(path, node_path, extend_fn, collision_fn, max_iterations=smooth,
                              max_time=max_time-elapsed_time(start_time))

            # Checking forward path
            flag = 0
            # input("check forward path...")
            print("check forward path...")
            pybullet_tools.utils.restore_state(start_state_id)
            for q in path_smoothed:
                if(collision_fn(q)):
                    # input("collision detected in forward path... Press enter to abandon current path...")
                    print("collision detected in forward path... Press enter to abandon current path...")
                    flag = 1
                    break

            if(flag == 0):
                # input("forward path checking completed. No issues found")
                print("forward path checking completed. No issues found")
                break # Break out of loop if the smoothed path is fine


        if(flag == 0):
            path = path_smoothed

        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions



def random_restarts_v2(solve_fn, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    movable, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Paths [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    solutions = []
    # path = check_direct(start, goal, extend_fn, collision_fn)
    # if path is False:
    #     return None
    # if path is not None:
    #     solutions.append(path)

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fn, movable,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue
        path = smooth_path_v2(path, extend_fn, collision_fn, max_iterations=smooth,
                           max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions



def direct_path_with_controls(robot, start, goal, extend_fn, collision_fn):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :return: Path [q', ..., q"] or None if unable to find a solution
    """
    # TODO: version which checks whether the segment is valid
    # if collision_fn(start) or collision_fn(goal):
    #     # TODO: return False
    #     return None
    path = list(extend_fn(start, goal))
    path = [start] + path

    joints = pybullet_tools.utils.get_movable_joints(robot)
    pybullet_tools.utils.set_joint_positions(robot, joints, start)
    pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, start, positionGains=7 * [0.01])

    for t in range(10):
        s_utils.step_sim()

    if any(collision_fn(q) for q in default_selector(path)):
        return None
    return path

def check_direct_with_controls(robot, start, goal, extend_fn, collision_fn):
    # if any(collision_fn(q) for q in [start, goal]):
    #     return False
    return direct_path_with_controls(robot, start, goal, extend_fn, collision_fn)


def random_restarts_with_controls(solve_fn, robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Paths [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    solutions = []
    path = check_direct_with_controls(robot, start, goal, extend_fn, collision_fn)
    if path is False:
        return None
    if path is not None:
        solutions.append(path)

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue
        path = smooth_path_with_controls(path, robot, extend_fn, collision_fn, max_iterations=smooth,
                           max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions


def random_restarts(solve_fn, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                    restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                    success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    :param start: Start configuration - conf
    :param goal: End configuration - conf
    :param distance_fn: Distance function - distance_fn(q1, q2)->float
    :param sample_fn: Sample function - sample_fn()->conf
    :param extend_fn: Extension function - extend_fn(q1, q2)->[q', ..., q"]
    :param collision_fn: Collision function - collision_fn(q)->bool
    :param max_time: Maximum runtime - float
    :param kwargs: Keyword arguments
    :return: Paths [[q', ..., q"], [[q'', ..., q""]]
    """
    start_time = time.time()
    solutions = []
    path = check_direct(start, goal, extend_fn, collision_fn)
    if path is False:
        return None
    if path is not None:
        solutions.append(path)

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue
        path = smooth_path(path, extend_fn, collision_fn, max_iterations=smooth,
                           max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions

def solve_and_smooth(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs):
    return random_restarts(solve_fn, q1, q2, distance_fn, sample_fn, extend_fn, collision_fn, restarts=0, **kwargs)

#################################################################

def solve(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, algorithm='birrt',
          max_time=INF, max_iterations=INF, num_samples=100, smooth=None, **kwargs):
    # TODO: allow distance_fn to be skipped
    # TODO: return lambda function
    start_time = time.time()
    path = check_direct(start, goal, extend_fn, collision_fn)
    if path is not None:
        return path
    #max_time -= elapsed_time(start_time)
    if algorithm == 'prm':
        path = prm(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   num_samples=num_samples)
    elif algorithm == 'lazy_prm':
        path = lazy_prm(start, goal, sample_fn, extend_fn, collision_fn,
                        num_samples=num_samples, max_time=max_time)[0]
    elif algorithm == 'rrt':
        path = rrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                   max_iterations=max_iterations, max_time=max_time)
    elif algorithm == 'rrt_connect':
        path = rrt_connect(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                           max_iterations=INF, max_time=max_time)
    elif algorithm == 'birrt':
        # TODO: checks the straight-line twice
        path = birrt(start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                     max_iterations=INF, max_time=max_time, smooth=None, **kwargs) # restarts=2
    elif algorithm == 'rrt_star':
        path = rrt_star(start, goal, distance_fn, sample_fn, extend_fn, collision_fn, radius=1,
                        max_iterations=INF, max_time=max_time)
    elif algorithm == 'lattice':
        path = lattice(start, goal, extend_fn, collision_fn, distance_fn=distance_fn, max_time=INF)
    else:
        raise NotImplementedError(algorithm)
    return smooth_path(path, extend_fn, collision_fn, max_iterations=smooth, max_time=max_time-elapsed_time(start_time))
