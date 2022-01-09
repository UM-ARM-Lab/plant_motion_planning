import pickle
import time

import numpy as np
import pybullet

import plant_motion_planning.pybullet_tools.utils
# from scripts.utils import step_sim
import plant_motion_planning.utils as s_utils
from .lattice import lattice
from .lazy_prm import lazy_prm
from .prm import prm
from .rrt import rrt
# from .rrt_connect import rrt_connect, birrt
from .rrt_star import rrt_star

from .smoothing import smooth_path, smooth_path_v2, smooth_path_v4, smooth_path_with_controls, smooth_path_v6, \
    smooth_path_v7, smooth_path_single_plant, smooth_path_multi_world, smooth_path_multiworld_benchmark
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


def random_restarts_single_plant(solve_fn, robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                  restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                                  success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    Function to plan using solve_fn and smooth path if found for a single stemmed plant environment.
    """
    start_time = time.time()
    solutions = []

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        
        # Find a path from start to goal using solve_fn algorithm
        path = solve_fn(robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue

        print("Path found! Press enter to start smoothing...")

        # Perform smoothing on noisy path
        path = smooth_path_single_plant(path, robot, extend_fn, collision_fn, max_iterations=smooth,
                                         max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions



def random_restarts_multiworld_benchmark(solve_fn, multi_world_env, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                  restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                                  success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    Path computation using solve_fn and smoothing of noisy path
    """
    start_time = time.time()
    solutions = []


    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))
        path = solve_fn(multi_world_env, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue

        print("Path found! Smoothing...")

        path = smooth_path_multiworld_benchmark(path, multi_world_env, extend_fn, collision_fn, max_iterations=smooth,
                                         max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions


def random_restarts_with_controls(solve_fn, robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                                  restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                                  success_cost=0., max_time=INF, max_solutions=1, **kwargs):
    """
    This function performs planning using solve_fn and then smoothing.
    """
    start_time = time.time()
    solutions = []

    for attempt in irange(restarts + 1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break
        attempt_time = (max_time - elapsed_time(start_time))

        # Planning step
        path = solve_fn(robot, start, goal, distance_fn, sample_fn, extend_fn, collision_fn,
                        max_time=attempt_time, **kwargs)
        if path is None:
            continue

        print("Path found! Press enter to start smoothing...")

        # Smoothing step
        path = smooth_path_with_controls(path, robot, extend_fn, collision_fn, max_iterations=smooth,
                                         max_time=max_time-elapsed_time(start_time))
        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break
    solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
        path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions



def random_restarts_multi_world(solve_fn, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                       multi_plant_env, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                       success_cost=0., max_time=INF, max_solutions=1, **kwargs):

    """
    Function that finds a path between the two given configurations, smoothes them and sorts candidate paths
    using the end effector distance travelled.

    :param solve_fn: Planner algorithm. eg. RRT connect
    :param robot: Body ID of robot
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param start: Initial or current configuration
    :param goal: Final goal configuration
    :param distance_fn: Distance metric
    :param sample_fn: Sampling function
    :param extend_fn: Extension function for RRT
    :param collision_fn: Collision function
    :param movable: list of movable entities that are represented by their characterization objects
    :param restarts: Number of times these attempts must be repeated.
    :param smooth: -
    :param success_cost: -
    :param max_time: -
    :param max_solutions: Maximum number of solutions that must be found before sorting
    :param kwargs: -
    :return:
    """

    start_time = time.time()
    solutions = []
    path_lengths = []

    smooth_attempts = 10
    max_iterations_per_smoothing = 20



    collision_fn, collision_fn_back = collision_fns

    while(1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break

        print("Calculating solution: %d" % (len(solutions) + 1))

        attempt_time = (max_time - elapsed_time(start_time))
        path_data = solve_fn(start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                             multi_plant_env, max_time=attempt_time, **kwargs)

        if path_data is None or path_data[0] is None:
            continue

        path, node_path = path_data

        path_cost = INF

        for smooth_attempt in range(smooth_attempts):
            path_smoothed = smooth_path_multi_world(path, node_path, extend_fn, collision_fn, multi_plant_env,
                                           max_iterations=max_iterations_per_smoothing,
                                           max_time=max_time-elapsed_time(start_time))

            # Checking forward path
            flag = 0

            print("check forward path...")
            print("=====================================")
            plant_motion_planning.pybullet_tools.utils.restore_state(start_state_id)

            for env in multi_plant_env.envs:
                plant_motion_planning.pybullet_tools.utils.set_joint_positions(multi_plant_env.envs[env].robot, multi_plant_env.joints, start)

            for t in range(2):
                multi_plant_env.step(start)

            print("press enter to check the forward path!")

            total_ee_path_cost = 0
            prev_ee_pos = np.array(pybullet.getLinkState(multi_plant_env.sample_robot, 9)[0])

            for q in path_smoothed:

                cur_ee_pos = np.array(pybullet.getLinkState(multi_plant_env.sample_robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                multi_plant_env.step(q)
                if(collision_fn(q)):

                    print("collision detected in forward path... Press enter to abandon current path...")

                    flag = 1
                    total_ee_path_cost = INF
                    break

            print("=====================================")

            if(flag == 0):
                print("forward path checking completed. No issues found")
                break # Break out of loop if the smoothed path is fine


        if(flag == 0):
            path = path_smoothed

        path_lengths.append(total_ee_path_cost)

        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break

    min_index = path_lengths.index(min(path_lengths))
    solutions = [solutions[min_index]]

    print("path lengths: ", path_lengths)

    print("Choosing solution: %d" % (min_index))

    return solutions


def random_restarts_v7(solve_fn, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                       single_plant_env, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                       success_cost=0., max_time=INF, max_solutions=1, **kwargs):

    """
    Function that finds a path between the two given configurations, smoothes them and sorts candidate paths
    using the end effector distance travelled.

    :param solve_fn: Planner algorithm. eg. RRT connect
    :param robot: Body ID of robot
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param start: Initial or current configuration
    :param goal: Final goal configuration
    :param distance_fn: Distance metric
    :param sample_fn: Sampling function
    :param extend_fn: Extension function for RRT
    :param collision_fn: Collision function
    :param movable: list of movable entities that are represented by their characterization objects
    :param restarts: Number of times these attempts must be repeated.
    :param smooth: -
    :param success_cost: -
    :param max_time: -
    :param max_solutions: Maximum number of solutions that must be found before sorting
    :param kwargs: -
    :return:
    """

    start_time = time.time()
    solutions = []
    path_lengths = []
    # path = check_direct(start, goal, extend_fn, collision_fn)
    # if path is False:
    #     return None
    # if path is not None:
    #     solutions.append(path)

    smooth_attempts = 5
    max_iterations_per_smoothing = 10

    # for attempt in irange(restarts + 1):


    collision_fn, collision_fn_back = collision_fns

    while(1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break

        print("Calculating solution: %d" % (len(solutions) + 1))

        attempt_time = (max_time - elapsed_time(start_time))
        path_data = solve_fn(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                             single_plant_env, max_time=attempt_time, **kwargs)

        if path_data is None or path_data[0] is None:
            continue

        path, node_path = path_data

        # else:
        #     path, node_path = path_data

        # input("Path found, press enter to start smoothing...")

        path_cost = INF
        # total_ee_path_cost = INF

        for smooth_attempt in range(smooth_attempts):
            path_smoothed = smooth_path_v7(robot, path, node_path, extend_fn, collision_fn, single_plant_env,
                                           max_iterations=max_iterations_per_smoothing,
                                           max_time=max_time-elapsed_time(start_time))

            # Checking forward path
            flag = 0
            # input("check forward path...")
            print("check forward path...")
            print("=====================================")
            plant_motion_planning.pybullet_tools.utils.restore_state(start_state_id)

            # plant_motion_planning.pybullet_tools.utils.set_joint_positions(robot, single_plant_env.joints, start)
            # pybullet.setJointMotorControlArray(robot, single_plant_env.joints, pybullet.POSITION_CONTROL, start,
            #                                    positionGains=7 * [0.01])

            # for t in range(2):
            #     s_utils.step_sim_v2()
            #     single_plant_env.step(start)
            single_plant_env.step(start)

            # input("press enter to check the forward path!")
            print("press enter to check the forward path!")

            # for q in path_smoothed:
            #
            #     pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, q, positionGains=7 * [0.01])
            #
            #     for t in range(10):
            #         s_utils.step_sim()
            #
            #     for e, b in enumerate(movable):
            #         b.observe()
            #
            #         print("Deflection of plant %d: %f" % (e, b.deflection))
            #
            #         if (b.deflection > 0.5):
            #             print("Error! Deflection limit exceeded!")
            #             input("")
            total_ee_path_cost = 0
            prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])
            for q in path_smoothed:

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                single_plant_env.step(q)
                if(collision_fn(q, True)):
                    # input("collision detected in forward path... Press enter to abandon current path...")
                    print("collision detected in forward path... Press enter to abandon current path...")
                    # for e, b in enumerate(movable):
                    #     b.observe()
                    #     print("Deflection of plant %d: %f" % (e, b.deflection))
                    # import pickle
                    # with open('path_smoothed.pkl', 'wb') as f:
                    #     pickle.dump([path_smoothed], f)

                    # input("")
                    flag = 1
                    total_ee_path_cost = INF
                    break

                # for e, b in enumerate(movable):
                #     b.observe()
                #     print("Deflection of plant %d: %f" % (e, b.deflection))

            print("=====================================")

            if(flag == 0):
                # input("forward path checking completed. No issues found")
                print("forward path checking completed. No issues found")
                break # Break out of loop if the smoothed path is fine


        if(flag == 0):
            path = path_smoothed

        path_lengths.append(total_ee_path_cost)

        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break

    min_index = path_lengths.index(min(path_lengths))
    solutions = [solutions[min_index]]

    print("path lengths: ", path_lengths)

    print("Choosing solution: %d" % (min_index))


    # solutions = sorted(solutions, key=lambda path: compute_path_cost(path, distance_fn))
    # print('Solutions ({}): {} | Time: {:.3f}'.format(len(solutions), [(len(path), round(compute_path_cost(
    #     path, distance_fn), 3)) for path in solutions], elapsed_time(start_time)))
    return solutions


def random_restarts_v6(solve_fn, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                       movable, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                       success_cost=0., max_time=INF, max_solutions=1, **kwargs):

    """
    Function that finds a path between the two given configurations, smoothes them and sorts candidate paths
    using the end effector distance travelled.

    :param solve_fn: Planner algorithm. eg. RRT connect
    :param robot: Body ID of robot
    :param start_state_id: The saved state ID of the initial state of the environment. This is returned by pybullet
    when saving the environment.
    :param start: Initial or current configuration
    :param goal: Final goal configuration
    :param distance_fn: Distance metric
    :param sample_fn: Sampling function
    :param extend_fn: Extension function for RRT
    :param collision_fn: Collision function
    :param movable: list of movable entities that are represented by their characterization objects
    :param restarts: Number of times these attempts must be repeated.
    :param smooth: -
    :param success_cost: -
    :param max_time: -
    :param max_solutions: Maximum number of solutions that must be found before sorting
    :param kwargs: -
    :return:
    """

    start_time = time.time()
    solutions = []
    path_lengths = []

    smooth_attempts = 5
    max_iterations_per_smoothing = 100


    collision_fn, collision_fn_back = collision_fns

    while(1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break

        print("Calculating solution: %d" % (len(solutions) + 1))

        attempt_time = (max_time - elapsed_time(start_time))
        
        # Find path from start to goal using solve_fn
        path_data = solve_fn(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fns, movable,
                             max_time=attempt_time, **kwargs)

        if path_data is None or path_data[0] is None:
            continue

        path, node_path = path_data

        path_cost = INF

        for smooth_attempt in range(smooth_attempts):
            # Attempt to smooth path
            path_smoothed = smooth_path_v6(robot, path, node_path, extend_fn, collision_fn,
                                           max_iterations=max_iterations_per_smoothing,
                                           max_time=max_time-elapsed_time(start_time))

            flag = 0

            # Checking forward path
            print("check forward path...")
            print("=====================================")
            plant_motion_planning.pybullet_tools.utils.restore_state(start_state_id)

            joints = plant_motion_planning.pybullet_tools.utils.get_movable_joints(robot)
            plant_motion_planning.pybullet_tools.utils.set_joint_positions(robot, joints, start)
            pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, start,
                                               positionGains=7 * [0.01])
            for t in range(20):
                s_utils.step_sim_v2()

            total_ee_path_cost = 0
            prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])
            for q in path_smoothed:

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                if(collision_fn(q, True)):
                    print("collision detected in forward path... Press enter to abandon current path...")

                    flag = 1
                    total_ee_path_cost = INF
                    break

            print("=====================================")

            if(flag == 0):
                print("forward path checking completed. No issues found")
                break # Break out of loop if the smoothed path is fine


        if(flag == 0):
            path = path_smoothed

        path_lengths.append(total_ee_path_cost)

        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break

    min_index = path_lengths.index(min(path_lengths))
    solutions = [solutions[min_index]]

    print("path lengths: ", path_lengths)

    print("Choosing solution: %d" % (min_index))

    return solutions


def random_restarts_v4(solve_fn, robot, start_state_id, start, goal, distance_fn, sample_fn, extend_fn, collision_fns,
                       movable, restarts=RRT_RESTARTS, smooth=RRT_SMOOTHING,
                       success_cost=0., max_time=INF, max_solutions=2, **kwargs):
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
    path_lengths = []

    smooth_attempts = 5
    max_iterations_per_smoothing = 10


    collision_fn, collision_fn_back  = collision_fns

    while(1):
        if (len(solutions) >= max_solutions) or (elapsed_time(start_time) >= max_time):
            break

        print("Calculating solution: %d" % (len(solutions) + 1))

        attempt_time = (max_time - elapsed_time(start_time))

        # Find path from start to goal
        path_data = solve_fn(robot, start, start_state_id, goal, distance_fn, sample_fn, extend_fn, collision_fns, movable,
                        max_time=attempt_time, **kwargs)

        if path_data is None or path_data[0] is None:
            continue

        path, node_path = path_data

        path_cost = INF

        for smooth_attempt in range(smooth_attempts):
            # Find smoothed path given a noisy path
            path_smoothed = smooth_path_v4(robot, path, node_path, extend_fn, collision_fn,
                                           max_iterations=max_iterations_per_smoothing,
                                           max_time=max_time-elapsed_time(start_time))

            # Checking forward path
            flag = 0
            print("check forward path...")
            print("=====================================")
            plant_motion_planning.pybullet_tools.utils.restore_state(start_state_id)

            joints = plant_motion_planning.pybullet_tools.utils.get_movable_joints(robot)
            plant_motion_planning.pybullet_tools.utils.set_joint_positions(robot, joints, start)
            pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, start,
                                               positionGains=7 * [0.01])
            for t in range(20):
                s_utils.step_sim()

            total_ee_path_cost = 0
            prev_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])
            for q in path_smoothed:

                cur_ee_pos = np.array(pybullet.getLinkState(robot, 9)[0])

                total_ee_path_cost = total_ee_path_cost + np.linalg.norm(cur_ee_pos - prev_ee_pos)

                if(collision_fn(q, True)):
                    print("collision detected in forward path... Press enter to abandon current path...")

                    flag = 1
                    total_ee_path_cost = INF
                    break


            print("=====================================")

            if(flag == 0):
                print("forward path checking completed. No issues found")
                break # Break out of loop if the smoothed path is fine


        if(flag == 0):
            path = path_smoothed

        path_lengths.append(total_ee_path_cost)

        solutions.append(path)
        if compute_path_cost(path, distance_fn) < success_cost:
            break

    min_index = path_lengths.index(min(path_lengths))
    solutions = [solutions[min_index]]

    print("path lengths: ", path_lengths)

    print("Choosing solution: %d" % (min_index))

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

    joints = plant_motion_planning.pybullet_tools.utils.get_movable_joints(robot)
    plant_motion_planning.pybullet_tools.utils.set_joint_positions(robot, joints, start)
    pybullet.setJointMotorControlArray(robot, joints, pybullet.POSITION_CONTROL, start, positionGains=7 * [0.01])

    for t in range(10):
        s_utils.step_sim()
    # for t in range(10):
    #     s_utils.step_sim_v2()

    if any(collision_fn(q) for q in default_selector(path)):
        return None
    return path

def check_direct_with_controls(robot, start, goal, extend_fn, collision_fn):
    # if any(collision_fn(q) for q in [start, goal]):
    #     return False
    return direct_path_with_controls(robot, start, goal, extend_fn, collision_fn)





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
