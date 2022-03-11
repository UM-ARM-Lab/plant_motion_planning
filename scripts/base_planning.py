from cmath import cos
import random
from plant_motion_planning.pybullet_tools.husky_utils import execute_path, gen_prims, get_cost_fn, get_running_cost_fn, get_dynamics_fn, \
     get_sample_fn, get_collision_fn
from plant_motion_planning.pybullet_tools.kinodynamic_rrt import rrt_solve
from plant_motion_planning.pybullet_tools.utils import disable_real_time, disconnect, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, wait_for_user
from pytorch_mppi import mppi
import torch
import pybullet as p

from plant_motion_planning.utils import step_sim

cli = connect(use_gui=True,width=1000, height=700)
disable_real_time()

# Set camera pose to desired position and orientation
set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))

#torch.seed(0)
random.seed(10)

# Draw X, Y, Z axes
#draw_global_system()

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
enable_gravity()
robot = load_model(HDT_MICHIGAN_URDF, pose=((0, 0, 0.31769884443141244), (0, 0, 0, 1)), fixed_base = False)

d = "cpu"

start = torch.tensor([[0., 0., 0., 0., 0.]], device=d)
goal = torch.tensor([[10., 5., 0., 0., 0.]], device=d)

p.addUserDebugText("Goal", goal[0, 0:3], textSize=1, textColorRGB=(0, 1, 0))

dynamics_fn = get_dynamics_fn()
sample_fn = get_sample_fn()
cost_fn = get_cost_fn()
running_cost_fn = get_running_cost_fn(goal)
collision_fn = get_collision_fn()

prims = gen_prims(13, dynamics_fn, d, False)
# path = rrt_solve(start, goal, dynamics_fn, collision_fn, cost_fn, sample_fn, prims)

for prim in prims:
     execute_path(robot, start, prim, dynamics_fn, draw_path=True)
# if path is not None:
#     input("Path found! Enter to execute")
#     execute_path(robot, start, path, dynamics_fn, True)
# else:
#     print("No path found")

wait_for_user()
disconnect()

