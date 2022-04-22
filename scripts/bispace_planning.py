import sys, os
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from cmath import cos
import random
from plant_motion_planning.pybullet_tools.val_utils import get_arm_joints
from plant_motion_planning.plant import Plant
from plant_motion_planning.pybullet_tools.husky_utils import HuskyUtils
from plant_motion_planning.pybullet_tools.environment import Environment, find_z_coord
from plant_motion_planning.pybullet_tools.bispace_rrt import bispace_rrt
from plant_motion_planning.pybullet_tools.utils import disable_gravity, disable_real_time, disconnect, draw_circle, get_movable_joints, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, HDT_MICHIGAN_SRDF, wait_for_user
from arm_pytorch_utilities import rand
import torch
import pybullet as p

from plant_motion_planning.utils import step_sim

cli = connect(use_gui=True,width=1000, height=700)
disable_real_time()

# Set camera pose to desired position and orientation
set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))
#736203 - close goal
#660773 - far goal
seed = rand.seed()
print("Seed being used: ", seed)

LEFT_EE_INDEX = 39
RIGHT_EE_INDEX = 55

is_left = True

# time step params
pybullet_period = 1 / 240.
time_multiplier = 50.

# Draw X, Y, Z axes
#draw_global_system()

device = "cpu"

p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(0, 0)

enable_gravity()
z_coord = find_z_coord()
disable_gravity()

env = Environment(offset=(0, 0), num_plants=(2, 2), z_coord=z_coord, num_steps=50, device=device)

arm_joints = get_arm_joints(env.robot, is_left=is_left, include_torso=False)
ee_link_idx = LEFT_EE_INDEX if is_left else RIGHT_EE_INDEX

xStart = torch.tensor([[-0.5, -0.5, 0., 0., 0.]], device=device)
qStart = torch.zeros(len(arm_joints), device=device)
#bGoal = torch.tensor([2.0, -0.5, 0.75, 0, 0, 0])
bGoal = torch.tensor([3.0, 2.0, 0.75, 0, 0, 0])

sphere = p.loadURDF('models/debug_sphere.urdf', basePosition=bGoal[0:3], useFixedBase=True)

husky_utils = HuskyUtils(env, floor, z_coord, pybullet_period*time_multiplier, device, arm_joints, ee_link_idx, HDT_MICHIGAN_SRDF)

dynamics_fn = husky_utils.get_dynamics_fn()
sample_fn = husky_utils.get_sample_fn()
sample_conf_step_fn = husky_utils.get_sample_conf_step_fn()
sample_goal_step_fn = husky_utils.get_sample_goal_step_fn()
base_cost_fn = husky_utils.get_base_cost_fn()
arms_cost_fn = husky_utils.get_arms_cost_fn()
goal_cost_fn = husky_utils.get_goal_cost_fn()
collision_fn = husky_utils.get_collision_fn()
connect_fn = husky_utils.get_connect_fn()
conf_to_goal_fn = husky_utils.get_ee_pose_fn()

print("start prims")
prims = husky_utils.gen_prims(num_prims=12, draw_prims=False)
print("finish prims")

def debug_fn(create_node=False, follow_node=False, unfollow_node=False, node_position=None, node_id=None):
    if create_node:
        if node_position is not None:
            id = p.loadURDF('models/debug_sphere.urdf', basePosition=node_position, useFixedBase=True, globalScaling=0.5)
            p.changeVisualShape(id, -1, rgbaColor=(0, 0, 1, 1))
            return id
    elif follow_node:
        if node_id is not None:
            p.changeVisualShape(node_id, -1, rgbaColor=(0, 1, 0, 1))
    elif unfollow_node:
        if node_id is not None:
            p.changeVisualShape(node_id, -1, rgbaColor=(0, 0, 1, 1))

init_state = p.saveState()
path = bispace_rrt((xStart, qStart), bGoal, dynamics_fn, connect_fn, collision_fn, env.save_state, env.restore_state, base_cost_fn, arms_cost_fn,
                    conf_to_goal_fn, goal_cost_fn, sample_fn, sample_conf_step_fn, sample_goal_step_fn, prims, debug_fn=debug_fn)
#p.restoreState(init_state)

if path is not None:
    input("Path found! Enter to execute")
    print("Number of path nodes: ", len(path))
    #log = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName='vid.mp4')
    husky_utils.execute_path(xStart, path, draw_path=True, color=(0, 0, 1))
    #p.stopStateLogging(log)
else:
    print("No path found")

wait_for_user()
disconnect()

