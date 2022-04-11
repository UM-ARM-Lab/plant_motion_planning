from cmath import cos
import random
from plant_motion_planning.pybullet_tools.val_utils import get_arm_joints
from plant_motion_planning.plant import Plant
from plant_motion_planning.pybullet_tools.husky_utils import HuskyUtils
from plant_motion_planning.pybullet_tools.environment import Environment, find_z_coord
from plant_motion_planning.pybullet_tools.kinodynamic_rrt import Node, rrt_solve
from plant_motion_planning.pybullet_tools.utils import disable_gravity, disable_real_time, disconnect, draw_circle, get_movable_joints, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, HDT_MICHIGAN_SRDF, wait_for_user
from pytorch_mppi import mppi
from arm_pytorch_utilities import rand
import torch
import pybullet as p

from plant_motion_planning.utils import step_sim

cli = connect(use_gui=True,width=1000, height=700)
disable_real_time()

# Set camera pose to desired position and orientation
set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))

# 381457: works well -> extend, prims when steering fails
seed = rand.seed()
print("Seed being used: ", seed)

# time step params
pybullet_period = 1 / 240.
time_multiplier = 50.

# Draw X, Y, Z axes
#draw_global_system()

device = "cpu"

start = torch.tensor([[-0.5, -0.5, 0., 0., 0.]], device=device)
goal = torch.tensor([[3.0, 2.0, torch.pi, 0., 0.]], device=device)

p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(0, 0)

enable_gravity()
z_coord = find_z_coord()
disable_gravity()

env = Environment(offset=(0, 0), num_plants=(2, 2), z_coord=z_coord, num_steps=50, device=device)

p.addUserDebugText("Goal", (goal[0, 0], goal[0, 1], 0), textSize=1, textColorRGB=(0, 1, 0))

arm_joints = get_arm_joints(env.robot, is_left=True, include_torso=False)
qStart = torch.zeros(len(arm_joints), device=device)
qGoal = torch.zeros(len(arm_joints), device=device)
qGoal[0] = 0.25
qGoal[1] = 0.15

husky_utils = HuskyUtils(env, floor, z_coord, pybullet_period*time_multiplier, device, arm_joints, HDT_MICHIGAN_SRDF)

dynamics_fn = husky_utils.get_dynamics_fn()
sample_fn = husky_utils.get_sample_fn()
base_cost_fn = husky_utils.get_base_cost_fn()
arms_cost_fn = husky_utils.get_arms_cost_fn()
collision_fn = husky_utils.get_collision_fn()
steering_fn = husky_utils.get_steering_fn()
connect_fn = husky_utils.get_connect_fn()

print("start prims")
prims = husky_utils.gen_prims(num_prims=12, draw_prims=True)
print("finish prims")

init_state = p.saveState()
path,old_path = rrt_solve((start, qStart), (goal, qGoal), dynamics_fn, steering_fn, connect_fn, collision_fn, env.save_state, env.restore_state,
                             base_cost_fn, arms_cost_fn, sample_fn, prims, husky_utils.execute_path)
p.restoreState(init_state)


if path is not None:
    input("Path found! Enter to execute")
    print("Number of path nodes: ", len(path))
    #log = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName='vid.mp4')
    husky_utils.execute_path(start, path, draw_path=True, color=(0, 0, 1))
    #p.stopStateLogging(log)
else:
    print("No path found")

wait_for_user()
disconnect()

