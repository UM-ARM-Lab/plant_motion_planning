from cmath import cos
import random
from plant_motion_planning.pybullet_tools.val_utils import get_arm_joints
from plant_motion_planning.plant import Plant
from plant_motion_planning.pybullet_tools.husky_utils import HuskyUtils
from plant_motion_planning.pybullet_tools.kinodynamic_rrt import rrt_solve
from plant_motion_planning.pybullet_tools.utils import disable_real_time, disconnect, draw_circle, load_model, enable_gravity, set_camera_pose, \
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

# Draw X, Y, Z axes
#draw_global_system()

device = "cpu"

start = torch.tensor([[-0.5, -0.5, 0., 0., 0.]], device=device)
goal = torch.tensor([[3., 2., torch.pi, 0., 0.]], device=device)

p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(0, 0)
enable_gravity()

GARDEN_WIDTH = 1.2192
GARDEN_LENGTH = 2.4384
GARDEN_BORDER = 0.1

wall1 = load_model("models/wall_short.urdf", fixed_base=True, pose=((0, GARDEN_WIDTH / 2, 0), (0, 0, 0, 1)))
wall2 = load_model("models/wall_short.urdf", fixed_base=True, pose=((GARDEN_LENGTH + GARDEN_BORDER, GARDEN_WIDTH / 2, 0), (0, 0, 0, 1)))
wall3 = load_model("models/wall_long.urdf", fixed_base=True, pose=((GARDEN_WIDTH, 0, 0), (0, 0, 0, 1)))
wall4 = load_model("models/wall_long.urdf", fixed_base=True, pose=((GARDEN_WIDTH + GARDEN_BORDER, GARDEN_WIDTH, 0), (0, 0, 0, 1)))

# Find z-coord for robot
robot = p.loadURDF(HDT_MICHIGAN_URDF, basePosition=(start[0, 0], start[0, 1], 5))
for t in range(1000):
    p.stepSimulation()
pose, quat = p.getBasePositionAndOrientation(robot)
z_coord = pose[2]
p.removeBody(robot)

# Load Val for real this time
robot = p.loadURDF(HDT_MICHIGAN_URDF, basePosition=(start[0, 0], start[0, 1], z_coord), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

# 441699
# 726175 - backwards
# 836801
#368572
#619864 - gets stuck
#381457 - for with smoothing - interesting case
#462712 - smoothing fails
#114699 - loops
seed = rand.seed(381457)
print("Seed being used: ", seed)

p.addUserDebugText("Goal", (goal[0, 0], goal[0, 1], 0), textSize=1, textColorRGB=(0, 1, 0))

arm_joints = get_arm_joints(robot, is_left=True, include_torso=False)
q = torch.zeros(len(arm_joints), device=device)

husky_utils = HuskyUtils(robot, floor, z_coord, device, arm_joints, HDT_MICHIGAN_SRDF)

p1 = Plant(1, 2, 1, (GARDEN_LENGTH / 3, GARDEN_WIDTH / 3))
p2 = Plant(1, 2, 1, (GARDEN_LENGTH / 3, 2 * GARDEN_WIDTH / 3))
p3 = Plant(1, 2, 1, (2 * GARDEN_LENGTH / 3, GARDEN_WIDTH / 3))
p4 = Plant(1, 2, 1, (2 * GARDEN_LENGTH / 3, 2 * GARDEN_WIDTH / 3))

dynamics_fn = husky_utils.get_dynamics_fn()
sample_fn = husky_utils.get_sample_fn()
base_cost_fn = husky_utils.get_base_cost_fn()
arms_cost_fn = husky_utils.get_arms_cost_fn()
collision_fn = husky_utils.get_collision_fn()
steering_fn = husky_utils.get_steering_fn()
connect_fn = husky_utils.get_connect_fn()

# for i in range(0, p.getNumJoints(robot)+1):
#     p.changeVisualShape(robot, i, rgbaColor=(0, 1, 0, 1))
#     input(i)
#     p.changeVisualShape(robot, i, rgbaColor=(1, 1, 1, 1))

print("start prims")
prims = husky_utils.gen_prims(num_prims=12, draw_prims=True)
print("finish prims")

path,old_path = rrt_solve((start, q), (goal, q), dynamics_fn, steering_fn, connect_fn, collision_fn, base_cost_fn, arms_cost_fn, sample_fn, prims, husky_utils.execute_path)

if path is not None:
    input("Path found! Enter to execute")
    print("Number of path nodes: ", len(path))
    husky_utils.execute_path(start, old_path, draw_path=True, color=(1, 0, 0))
    #log = p.startStateLogging(loggingType=p.STATE_LOGGING_VIDEO_MP4, fileName='vid.mp4')
    husky_utils.execute_path(start, path, draw_path=True, color=(0, 0, 1))
    #p.stopStateLogging(log)
else:
    print("No path found")

wait_for_user()
disconnect()

