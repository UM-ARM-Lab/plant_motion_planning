from cmath import cos
import random
from plant_motion_planning.pybullet_tools.husky_utils import execute_path, gen_prims, get_cost_fn, get_running_cost_fn, get_dynamics_fn, \
     get_sample_fn, get_collision_fn
from plant_motion_planning.pybullet_tools.kinodynamic_rrt import rrt_solve
from plant_motion_planning.pybullet_tools.utils import disable_real_time, disconnect, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, wait_for_user
from pytorch_mppi import mppi
import numpy as np
import torch
import pybullet as p

from plant_motion_planning.utils import step_sim

cli = connect(use_gui=True,width=1000, height=700)
disable_real_time()

# Set camera pose to desired position and orientation
set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))

np.random.seed(10)
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
running_cost_fn = get_running_cost_fn(dynamics_fn, cost_fn, goal)
collision_fn = get_collision_fn()

noise_sigma = torch.tensor([[1., 0.], [0., 1.]], device=d)
ctrl = mppi.MPPI(dynamics_fn, running_cost_fn, 5, noise_sigma, device=d, u_min=torch.tensor([-1, -1]), u_max=torch.tensor([1, 1]))
x = start
path = []
for i in range(100):
     u = ctrl.command(x)
     x = dynamics_fn(x, u)
     path.append(u)
execute_path(robot, start, path, dynamics_fn, goal, draw_path=False)


# prims = gen_prims(15, dynamics_fn, False)
# path = rrt_solve(start, goal, dynamics_fn, collision_fn, cost_fn, sample_fn, prims)

# if path is not None:
#     input("Path found! Enter to execute")
#     execute_path(robot, start, path, dynamics_fn, True)
# else:
#     print("No path found")

wait_for_user()
disconnect()

