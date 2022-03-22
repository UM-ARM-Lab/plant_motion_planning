from cmath import cos
import random
from plant_motion_planning.pybullet_tools.husky_utils import HuskyUtils
from plant_motion_planning.pybullet_tools.kinodynamic_rrt import rrt_solve
from plant_motion_planning.pybullet_tools.utils import disable_real_time, disconnect, draw_circle, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, wait_for_user
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

p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(0, 0)
enable_gravity()

# Find z-coord for robot
robot = load_model(HDT_MICHIGAN_URDF, pose=((0, 0, 5), (0, 0, 0, 1)), fixed_base = False)
for t in range(1000):
    p.stepSimulation()
pose, quat = p.getBasePositionAndOrientation(robot)
z_coord = pose[2]
p.removeBody(robot)

# Load Val for real this time
robot = load_model(HDT_MICHIGAN_URDF, pose=((0, 0, z_coord), (0, 0, 0, 1)), fixed_base=True)

device = "cpu"

# 441699
# 726175 - backwards
seed = rand.seed()
print("Seed being used: ", seed)

start = torch.tensor([[0., 0., 0., 0., 0.]], device=device)
goal = torch.tensor([[10., 5., 0., 0., 0.]], device=device)

p.addUserDebugText("Goal", goal[0, 0:3], textSize=1, textColorRGB=(0, 1, 0))

husky_utils = HuskyUtils(robot, floor, z_coord, device)

dynamics_fn = husky_utils.get_dynamics_fn()
sample_fn = husky_utils.get_sample_fn()
cost_fn = husky_utils.get_cost_fn()
collision_fn = husky_utils.get_collision_fn()
steering_fn = husky_utils.get_steering_fn()

print("start prims")
prims = husky_utils.gen_prims(num_prims=12)
print("finish prims")
path, old_path = rrt_solve(start, goal, dynamics_fn, steering_fn, collision_fn, cost_fn, sample_fn, prims)

if path is not None:
    input("Path found! Enter to execute")
    print("Number of path nodes: ", len(path))
    husky_utils.execute_path(start, old_path, draw_path=True, color=(1, 0, 0))
    husky_utils.execute_path(start, path, draw_path=True, color=(0, 0, 1))
else:
    print("No path found")

wait_for_user()
disconnect()

