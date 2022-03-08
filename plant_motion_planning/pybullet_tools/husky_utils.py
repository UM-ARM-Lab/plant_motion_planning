import numpy as np
import pybullet as p
import rospkg
import torch
import yaml
from arm_pytorch_utilities.math_utils import angular_diff_batch

from plant_motion_planning.pybullet_tools.utils import step_simulation, wait_for_duration

r = rospkg.RosPack()
with open(r.get_path('husky_control') + '/config/control.yaml') as stream:
    parsed_yaml = yaml.safe_load(stream)

linear_params = parsed_yaml['husky_velocity_controller']['linear']['x']
angular_params = parsed_yaml['husky_velocity_controller']['angular']['z']

MAX_VELOCITY = torch.tensor([linear_params['max_velocity'], angular_params['max_velocity']])
MAX_ACCLERATION = torch.diag(torch.tensor([linear_params['max_acceleration'], angular_params['max_acceleration']]))
TIME_STEP = 1./240. * 50
STATE_DIM = 5
CONTROL_DIM = 2

MIN_X = -5
MIN_Y = -5
MIN_YAW = 0

MAX_X = 5
MAX_Y = 5
MAX_YAW = 2 * np.pi

MIN_LIMITS = torch.tensor([MIN_X, MIN_Y, MIN_YAW, -MAX_VELOCITY[0], -MAX_VELOCITY[1]]).T
MAX_LIMITS = torch.tensor([MAX_X, MAX_Y, MAX_YAW, MAX_VELOCITY[0], MAX_VELOCITY[0]]).T

def get_dynamics_fn():
    # Dynamics model for husky
    # x = [xpos, ypos, yaw, v, w]^T
    # u = [u1, u2] where u1, u2 [-1, 1] representing -1 as max deaccleration and 1 as max acceleration
    def dynamics_fn(x, u):
        # u has to be in range

        # if np.max(u) > 1 or np.min(u) < -1:
        #     return None
        
        accel = torch.matmul(u, MAX_ACCLERATION) * TIME_STEP
        new_vels = x[:, 3:5] + accel
        new_vels[:, 0] = torch.clamp(new_vels[:, 0], -MAX_VELOCITY[0], MAX_VELOCITY[1])
        new_vels[:, 1] = torch.clamp(new_vels[:, 1], -MAX_VELOCITY[1], MAX_VELOCITY[1])

        delta_distances = new_vels[:, 0:1] * TIME_STEP
        delta_poses = torch.cat((delta_distances * torch.cos(x[:, 2:3]), delta_distances * torch.sin(x[:, 2:3]), new_vels[:, 1:2] * TIME_STEP), 1)
        new_poses = x[:, 0:3] + delta_poses

        xnew = torch.cat((new_poses, new_vels), 1)
        return xnew
    return dynamics_fn

def execute_path(robot, start, path, dynamics_fn, goal, draw_path = False):
    x = start
    if draw_path:
        prev_x = x
        count = 0
    for u in path:
        x = dynamics_fn(x, u)
        set_pose(robot, x)
        if draw_path and not count % 5:
            draw_path_line(prev_x, x)
            prev_x = x

        wait_for_duration(TIME_STEP)
        cost = get_cost_fn()

def gen_prims(num_quarter_prims, dynamics_fn, draw_prims=False):
    # End conditions for generating primitives
    endX = 1.5
    endTheta = np.pi/2

    u = np.ones(2)
    prims = list() # list of primitives

    # Multiplies for generating prims in all 4 quadrants
    mult1 = np.array([1, 1]).T
    mult2 = np.array([1, -1]).T
    mult3 = np.array([-1, 1]).T
    mult4 = np.array([-1, -1]).T

    halfway = (int)(num_quarter_prims / 2) # When to switch to decreasing angular control

    # Add primitives from within 4 quadrants
    # Vary linear velocity first, then vary angular velocity
    for i in range(0, num_quarter_prims):
        x = np.zeros(STATE_DIM)
        prev_x = x
        prim1 = list()
        prim2 = list()
        prim3 = list()
        prim4 = list()

        counter = 0

        # Keep applying same control input untill primitive is generated
        while x[0] < endX and x[2] < endTheta:
            counter += 1
            x = dynamics_fn(x, u)

            # Make sure to capture all possible controls
            prim1.append(np.multiply(u, mult1))
            prim2.append(np.multiply(u, mult2))
            prim3.append(np.multiply(u, mult3))
            prim4.append(np.multiply(u, mult4))

            if draw_prims and not (counter % 50):
                draw_path_line(prev_x, x, (0, 0, 1))
                prev_x = x
        
        # Add primitives to list
        prims.append(prim1)
        prims.append(prim2)
        prims.append(prim3)
        prims.append(prim4)

        # Reset controls to max at halfway to prep for switch to angular
        if i == halfway:
            u = np.ones(2)

        # Decrease corresponding control input (linear is first half angular is second half) 
        # proportionally to the number of primitives we're generating
        index = 0 if (i < halfway) else 1
        u[index] -= (1 / (halfway+1))

    # Add 2 straight motion primitives (one straight forward and one straight back)
    u = np.zeros(2)
    u[0] = 1
    x = np.zeros(STATE_DIM)
    prev_x = x
    prim1 = list()
    prim2 = list()

    while x[0] < endX:
        x = dynamics_fn(x, u)

        prim1.append(np.multiply(u, mult1))
        prim2.append(np.multiply(u, mult3))

        if draw_prims:
            draw_path_line(prev_x, x, (0, 0, 1))
            prev_x = x

    prims.append(prim1)
    prims.append(prim2)

    # Add 2 rotating motion primitives (one counterclockwise and one clockwise)
    u = np.zeros(2)
    u[1] = 1
    x = np.zeros(STATE_DIM)
    prim1 = list()
    prim2 = list()

    while x[2] < endTheta:
        x = dynamics_fn(x, u)
        prim1.append(np.multiply(u, mult1))
        prim2.append(np.multiply(u, mult2))
        
    prims.append(prim1)
    prims.append(prim2)

    return prims

def get_collision_fn():
    # TODO write actual collision 
    def collision_fn(x):
        return False
    return collision_fn

def get_sample_fn():
    def sample_fn():
        return np.random.uniform(MIN_LIMITS, MAX_LIMITS)
    return sample_fn

def get_cost_fn():
    Q = torch.tensor([1, 1, 0.4, 0.1, 0.1])

    def cost_fn(x0, x1):
        diff = x0 - x1
        diff[:, 2] = angular_diff_batch(x0[:, 2], x1[:, 2])
        # diff[:, 2] = torch.minimum(torch.abs(diff[:, 2]), 2*torch.pi - torch.abs(diff[:, 2]))
        cost = diff ** 2 @ Q
        return cost
    return cost_fn

# For use with MPPI controller
def get_running_cost_fn(dynamics_fn, cost_fn, goal):
    R = 0.01

    def running_cost_fn(x, u):
        cost = u.norm(dim=1) * R + cost_fn(x, torch.matmul(torch.ones((x.size(dim=0), 1)), goal))
        return cost
    return running_cost_fn

def draw_path_line(x1, x2, color=(0, 0, 1), width = 1.0):
    draw_line((x1[0], x1[1], 0.01), (x2[0], x2[1], 0.01), width, color)

def draw_line(start, end, width, color=(0, 0, 1)):
    line_id = p.addUserDebugLine(start, end, color, width)
    return line_id

def set_pose(robot, x):
    position = (x[0, 0], x[0, 1], 0.31769884443141244)
    rotation = p.getQuaternionFromEuler((0, 0, x[0, 2]))
    p.resetBasePositionAndOrientation(robot, position, rotation)