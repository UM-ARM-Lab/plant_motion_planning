from random import sample
from plant_motion_planning.pybullet_tools.husky_utils import dynamics, execute_path, gen_prims, get_cost_fn, get_sample_fn
from plant_motion_planning.pybullet_tools.utils import disable_real_time, disconnect, load_model, enable_gravity, set_camera_pose, \
     step_simulation, connect, draw_global_system, HDT_MICHIGAN_URDF, wait_for_user
import numpy as np
import pybullet as p

from plant_motion_planning.utils import step_sim

cli = connect(use_gui=True,width=1000, height=700)
disable_real_time()

# Set camera pose to desired position and orientation
set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))



# Draw X, Y, Z axes
#draw_global_system()

p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)
enable_gravity()
#robot = load_model(HDT_MICHIGAN_URDF, pose=((0, 0, 0.31769884443141244), (0, 0, 0, 1)), fixed_base = False)

sample_fn = get_sample_fn()
cost_fn = get_cost_fn()

for i in range(10):
    x0 = sample_fn()
    x1 = sample_fn()
    cost = cost_fn(x0, x1)
    print(i, x0, x1, cost)

prims = gen_prims(5, False)
print(len(prims))

wait_for_user()
disconnect()

