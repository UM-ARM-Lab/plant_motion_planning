import random
import numpy as np
import time

from collections import OrderedDict, defaultdict
from itertools import combinations

from motion_planners.motion_planners.trajectory.linear import solve_multi_linear, quickest_inf_accel
from motion_planners.motion_planners.trajectory.limits import check_spline
from motion_planners.motion_planners.utils import waypoints_from_path, default_selector, irange
from motion_planners.motion_planners.trajectory.discretize import time_discretize_curve
from motion_planners.motion_planners.trajectory.smooth import smooth_curve

from pybullet_tools.utils import *
from pybullet_tools.retime import interpolate_path, sample_curve


BASE_LINK_NAME = "base_link"
BASE_JOINTS = ['x','y','theta']
DRAW_Z = 1e-3
DRAW_LENGTH = 0.5
MIN_AABB_VOL = DEFAULT_AABB_BUFFER**3

MAX_VELOCITIES = np.array([1,1,np.pi/4])
MAX_ACCELERATIONS = MAX_VELOCITIES / 0.25

MIN_PROXIMITY = 1e-2
N_DIGITS = 5

def create_custom_base_limits(robot, base_limits):
    return {joint_from_name(robot, joint): limits
            for joint, limits in safe_zip(BASE_JOINTS[:2], zip(*base_limits))}

def sample_placements(body_surfaces, obstacles=None, savers=[], min_distances={}):
    if obstacles is None:
        obstacles = set(get_bodies()) - set(body_surfaces)
    savers = list(savers) + [BodySaver(obstacle) for obstacle in obstacles]
    if not isinstance(min_distances, dict):
        min_distances = {body: min_distances for body in body_surfaces}
    # TODO: max attempts here
    for body, surface in body_surfaces.items(): # TODO: shuffle
        min_distance = min_distances.get(body, 0.)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                for saver in savers:
                    saver.restore()
                return False
            for saver in savers:
                obstacle = saver.body
                if obstacle in [body, surface]:
                    continue
                saver.restore()
                if pairwise_collision(body, obstacle, max_distance=min_distance):
                    break
            else:
                savers.append(BodySaver(body))
                break
    for saver in savers:
        saver.restore()
    return True

def simulate_curve(robot, joints, curve):
    #set_joint_positions(robot, joints, curve(random.uniform(curve.x[0], curve.x[-1])))
    wait_if_gui(message='Begin?')
    #controller = follow_curve_old(robot, joints, curve)
    controller = follow_curve(robot, joints, curve)
    for _ in controller:
        step_simulation()
        #wait_if_gui()
        #wait_for_duration(duration=time_step)
        #time.sleep(time_step)
    wait_if_gui(message='Finish?')

def step_curve(robot, joints, path, step_size=None):
    wait_if_gui(message='Begin?')
    for i, conf in enumerate(path):
        set_joint_positions(robot, joints, conf)
        if step_size is None:
            wait_if_gui(message='{}/{} Continue?'.format(i, len(path)))
        else:
            wait_for_duration(duration=step_size)
    wait_if_gui(message='Finish?')

def follow_curve(robot, joints, curve, goal_t=None, time_step=None, max_velocities=MAX_VELOCITIES, **kwargs):
    if goal_t is None:
        goal_t = curve.x[-1]
    if time_step is None:
        time_step = 10*get_time_step()
    #distance_fn = get_distance_fn(robot, joints, weights=None, norm=2)
    distance_fn = get_duration_fn(robot, joints, velocities=max_velocities, norm=INF) # get_distance
    positions = np.array(get_joint_positions(robot, joints))
    closest_dist, closest_t = find_closest(positions, curve, t_range=(curve.x[0], goal_t), max_time=1e-1,
                                           max_iterations=INF, distance_fn=distance_fn, verbose=True)
    print('Closest dist: {:.3f} | Closest time: {:.3f}'.format(closest_dist, closest_t))
    target_t = closest_t
    # TODO: condition based on closest_dist
    while True:
        print('\nTarget time: {:.3f} | Goal time: {}'.format(target_t, goal_t))
        target_positions = curve(target_t)
        target_velocities = curve(target_t, nu=1) # TODO: draw the velocity
        #print('Positions: {} | Velocities: {}'.format(target_positions, target_velocities))
        handles = draw_waypoint(target_positions)
        is_goal = (target_t == goal_t)
        position_tol = 1e-2 if is_goal else 1e-2
        for output in control_state(robot, joints, target_positions=target_positions, target_velocities=target_velocities,
                                    position_tol=position_tol, velocity_tol=INF, max_velocities=max_velocities, **kwargs):
            yield output
        remove_handles(handles)
        target_t = min(goal_t, target_t + time_step)
        if is_goal:
            break


def find_closest(x0, curve, t_range=None, max_time=INF, max_iterations=INF, distance_fn=None, verbose=False):
    assert (max_time < INF) or (max_iterations < INF)
    if t_range is None:
        t_range = Interval(curve.x[0], curve.x[-1])
    t_range = Interval(max(t_range[0], curve.x[0]), min(curve.x[-1], t_range[-1]))
    if distance_fn is None:
        distance_fn = get_distance
    start_time = time.time()
    closest_dist, closest_t = INF, None
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        t = random.uniform(*t_range) # TODO: halton
        x = curve(t)
        dist = distance_fn(x0, x)
        if dist < closest_dist:
            closest_dist, closest_t = dist, t
            if verbose:
                print('Iteration: {} | Dist: {:.3f} | T: {:.3f} | Time: {:.3f}'.format(
                    iteration, closest_dist, t, elapsed_time(start_time)))
    return closest_dist, closest_t


def control_state(robot, joints, target_positions, target_velocities=None, position_tol=INF, velocity_tol=INF,
                  max_velocities=None, verbose=True): # TODO: max_accelerations
    if target_velocities is None:
        target_velocities = np.zeros(len(joints))
    if max_velocities is None:
        max_velocities = get_max_velocities(robot, joints)
    assert (max_velocities > 0).all()
    max_velocity_control_joints(robot, joints, positions=target_positions, velocities=target_velocities,
                                max_velocities=max_velocities)
    for i in irange(INF):
        current_positions = np.array(get_joint_positions(robot, joints))
        position_error = get_distance(current_positions, target_positions, norm=INF)
        current_velocities = np.array(get_joint_velocities(robot, joints))
        velocity_error = get_distance(current_velocities, target_velocities, norm=INF)
        if verbose:
            # print('Positions: {} | Target positions: {}'.format(current_positions.round(N_DIGITS), target_positions.round(N_DIGITS)))
            # print('Velocities: {} | Target velocities: {}'.format(current_velocities.round(N_DIGITS), target_velocities.round(N_DIGITS)))
            print('Step: {} | Position error: {:.3f}/{:.3f} | Velocity error: {:.3f}/{:.3f}'.format(
                i, position_error, position_tol, velocity_error, velocity_tol))
        # TODO: draw the tolerance interval
        if (position_error <= position_tol) and (velocity_error <= velocity_tol):
            return # TODO: declare success or failure by yielding or throwing an exception
        yield i

def max_velocity_control_joints(robot, joints, positions=None, velocities=None, max_velocities=None):
    if velocities is None:
        velocities = np.zeros(len(joints))
    if max_velocities is None:
        max_velocities = get_max_velocities(robot, joints)
    for idx, joint in enumerate(joints):
        if positions is not None:
            control_joint(robot, joint=joint, position=positions[idx],
                          # velocity=0.,
                          velocity=velocities[idx], # if abs(velocities[idx]) > 1e-3 else 0,
                          # max_velocity=abs(velocities[idx]),
                          max_velocity=abs(max_velocities[idx]), # TODO: max_velocity and velocity==0 cause issues
                          position_gain=10, velocity_scale=None, max_force=None)
        else:
            velocity_control_joint(robot, joint=joint, velocity=velocities[idx],
                                   max_velocity=abs(max_velocities[idx]),
                                   position_gain=None, velocity_scale=None, max_force=None)




def draw_waypoint(conf, z=DRAW_Z):
    return draw_pose(pose_from_pose2d(conf, z=z), length=DRAW_LENGTH)

def get_curve_collision_fn(robot, joints, custom_limits={}, resolutions=None, v_max=None, a_max=None, **kwargs):
    collision_fn = get_collision_fn(robot, joints, custom_limits=custom_limits, **kwargs)
    limits_fn = get_limits_fn(robot, joints, custom_limits)

    def curve_collision_fn(curve, t0, t1):
        if curve is None:
            return True
        # TODO: can exactly compute limit violations
        # if not check_spline(curve, v_max=max_velocities, a_max=None, verbose=False,
        #                     #start_t=t0, end_t=t1,
        #                     ):
        #     return True
        _, samples = time_discretize_curve(curve, verbose=False,
                                           #start_t=t0, end_t=t1,
                                           resolution=resolutions,
                                           #max_velocities=v_max,
                                           )
        if any(map(limits_fn, samples)):
           return True
        if any(map(collision_fn, default_selector(samples))):
           return True
        return False
    return curve_collision_fn


def problem1(n_obs = 10,wall_side = 0.1, obs_w = 0.25,obs_h = 0.25):

    floor_extent = 5
    base_limits = (-floor_extent/2. * np.ones(2),floor_extent / 2. * np.ones(2))
    floor_height = 0.001

    floor = create_box(floor_extent, floor_extent, floor_height, color = TAN)
    set_point(floor, Point(z = -floor_height / 2.))

    wall1 = create_box(floor_extent + wall_side, wall_side, wall_side, color = GREY)
    set_point(wall1, Point(y = floor_extent/2., z = wall_side / 2.))
    wall2 = create_box(floor_extent + wall_side, wall_side, wall_side, color = GREY)
    set_point(wall2, Point(y = -floor_extent/2., z = wall_side/2.))
    wall3 = create_box(wall_side, floor_extent + wall_side, wall_side, color = GREY)
    set_point(wall3, Point(x = floor_extent/2., z = wall_side/2.))
    wall4 = create_box(wall_side, floor_extent + wall_side, wall_side, color = GREY)
    set_point(wall4, Point(x = -floor_extent/2., z = wall_side/2.))
    walls = [wall1, wall2, wall3, wall4]

    initial_surfaces = OrderedDict()
    for _ in range(n_obs):
        body = create_box(obs_w, obs_w, obs_h, color = GREY)
        initial_surfaces[body] = floor

    pillars = list(initial_surfaces)
    obstacles = walls + pillars


    initial_conf = np.array([+floor_extent/3., -floor_extent/3., 3*np.pi/4.])
    goal_conf = -initial_conf

    robot = load_pybullet(TURTLEBOT_URDF, merge = True, fixed_base = True, sat = False)
    base_joints = joints_from_names(robot, BASE_JOINTS)
    base_link = link_from_name(robot, BASE_LINK_NAME)

    set_all_color(robot, BLUE)
    dump_body(robot)

    set_point(robot, Point(z = stable_z(robot, floor)))

    draw_pose(Pose(), parent = robot, parent_link = base_link, length = 0.5)
    set_joint_positions(robot, base_joints, initial_conf)

    sample_placements(initial_surfaces, obstacles = [robot] + walls, savers = [BodySaver(robot, joints = base_joints, positions = goal_conf)], min_distances = 10e-2)

    return robot, base_limits, goal_conf, obstacles


def plan_motion(robot, joints, goal_positions, attachments=[], obstacles=None, holonomic=False, reversible=False, **kwargs):
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [robot] + attached_bodies
    if obstacles is None:
        obstacles = get_bodies()
    obstacles = set(obstacles) - set(moving_bodies)
    if holonomic:
        return plan_joint_motion(robot, joints, goal_positions,
                                 attachments=attachments, obstacles=obstacles, **kwargs)
    # TODO: just sample the x, y waypoint and use the resulting orientation
    # TODO: remove overlapping configurations/intervals due to circular joints
    return plan_nonholonomic_motion(robot, joints, goal_positions, reversible=reversible,
                                    linear_tol=1e-6, angular_tol=0.,
                                    attachments=attachments, obstacles=obstacles, **kwargs)


def compute_cost(robot, joints, path, resolutions=None):
    if path is None:
        return INF
    distance_fn = get_distance_fn(robot, joints, weights=resolutions) # TODO: get_duration_fn
    return sum(distance_fn(*pair) for pair in get_pairs(path))

def iterate_path(robot, joints, path, simulate=True, step_size=None, resolutions=None, smooth=True, **kwargs): # 1e-2 | None
    if path is None:
        return
    path = adjust_path(robot, joints, path)
    with LockRenderer():
        handles = draw_path(path)

    waypoints = path
    #waypoints = waypoints_from_path(path)
    #curve = interpolate_path(robot, joints, waypoints, k=1, velocity_fraction=1) # TODO: no velocities in the URDF

    curve = solve_multi_linear(waypoints, v_max=MAX_VELOCITIES, a_max=MAX_ACCELERATIONS)
    _, path = time_discretize_curve(curve, verbose=False, resolution=resolutions) # max_velocities=v_max,
    print('Steps: {} | Start: {:.3f} | End: {:.3f} | Knots: {}'.format(
        len(path), curve.x[0], curve.x[-1], len(curve.x)))
    with LockRenderer():
        handles = draw_path(path)

    if smooth:
        # TODO: handle circular joints
        #curve_collision_fn = lambda *args, **kwargs: False
        curve_collision_fn = get_curve_collision_fn(robot, joints, resolutions=resolutions, **kwargs)
        with LockRenderer():
            with BodySaver(robot):
                curve = smooth_curve(curve, MAX_VELOCITIES, MAX_ACCELERATIONS, curve_collision_fn, max_time=5) #, curve_collision_fn=[])
        path = [conf for t, conf in sample_curve(curve, time_step=step_size)]
        print('Steps: {} | Start: {:.3f} | End: {:.3f} | Knots: {}'.format(
            len(path), curve.x[0], curve.x[-1], len(curve.x)))
        with LockRenderer():
            handles = draw_path(path)

    if simulate:
        simulate_curve(robot, joints, curve)
    else:
        path = [conf for t, conf in sample_curve(curve, time_step=step_size)]
        step_curve(robot, joints, path, step_size=step_size)

def extract_full_path(robot, path_joints, path, all_joints):
    with BodySaver(robot):
        new_path = []
        for conf in path:
            set_joint_positions(robot, path_joints, conf)
            new_path.append(get_joint_positions(robot, all_joints)) # TODO: do without assigning
        return new_path

def draw_path(path2d, z=DRAW_Z, **kwargs):
    if path2d is None:
        return []
    #return list(flatten(draw_pose(pose_from_pose2d(pose2d, z=z), **kwargs) for pose2d in path2d))
    #return list(flatten(draw_pose2d(pose2d, z=z, **kwargs) for pose2d in path2d))
    base_z = 1.
    start = path2d[0]
    mid_yaw = start[2]
    #mid_yaw = wrap_interval(mid_yaw)
    interval = (mid_yaw - PI, mid_yaw + PI)
    #interval = CIRCULAR_LIMITS
    draw_pose(pose_from_pose2d(start, z=base_z), length=1, **kwargs)
    # TODO: draw the current pose
    # TODO: line between orientations when there is a jump
    return list(flatten(draw_conf(pose2d, interval, base_z, **kwargs) for pose2d in path2d))

def draw_conf(pose2d, interval, base_z, **kwargs):
    return draw_pose2d(pose2d, z=base_z + rescale_interval(
        wrap_interval(pose2d[2], interval=interval), old_interval=interval, new_interval=(-0.5, 0.5)), **kwargs)


def main():

    connect(use_gui = True)

    seed = None

    if(seed is None):
        seed = random.randint(0,10**3 - 1)

    print("seed used: ",seed)

    set_random_seed(seed = seed)
    set_numpy_seed(seed = seed)

    robot, base_limits, goal_conf, obstacles = problem1(n_obs = 10)
    custom_limits = create_custom_base_limits(robot, base_limits)
    base_joints = joints_from_names(robot, BASE_JOINTS)

    draw_base_limits(base_limits)

    start_conf = get_joint_positions(robot, base_joints)
    for conf in [start_conf, goal_conf]:
        draw_waypoint(conf)

    resolutions = 1. * DEFAULT_RESOLUTION * np.ones(len(base_joints))
    plan_joints = base_joints
    holonomic = True

    obstacles = []

    saver = WorldSaver()
    start_time = time.time()
    profiler = Profiler(field = 'cumtime', num = 50)
    profiler.save()

    with LockRenderer(lock = False):
        path = plan_motion(robot, plan_joints, goal_conf[:len(plan_joints)], holonomic = holonomic,
                           obstacles = obstacles, self_collisions = False,
                           custom_limts = custom_limits, resolutions = resolutions[:len(plan_joints)],
                           use_aabb = True, cache = True, max_distance = MIN_PROXIMITY,
                           algorithm = 'rrt_connect', restarts = 5, max_iterations = 50, smooth = 20)

        saver.restore()

    profiler.restore()

    solved = path is not None
    length = INF if path is None else len(path)
    cost = compute_cost(robot, base_joints, path, resolutions = resolutions[:len(plan_joints)])
    print('Solved: {} | Length: {} | Cost: {:.3f} | Runtime: {:.3f} sec'.format(
        solved, length, cost, elapsed_time(start_time)))
    if path is None:
        wait_if_gui()
        disconnect()
        return

    path = extract_full_path(robot, plan_joints, path, base_joints)
    iterate_path(robot, base_joints, path, step_size = 2e-2, custom_limits = custom_limits, resolutions = resolutions,
                 obstacles = obstacles, self_collisions = False, max_distance = MIN_PROXIMITY)

    disconnect()

if __name__ == '__main__':
    main()
