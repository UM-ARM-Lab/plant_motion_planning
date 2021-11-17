import numpy as np
import pybullet
import pybullet as p
# from pybullet_tools.utils import set_pose, Pose, Point, stable_z, step_simulation
import pybullet_tools.utils as pyb_tools_utils
from plant_motion_planning import representation

def set_random_pose(plant_id, floor, px = None, py = None):

    if px is None and py is None:
        px = 0.0
        py = 0.0
        while(px < 0.1 and py > -0.1):
            px = np.random.rand() * (0.6) - 0.2
            py = np.random.rand() * (0.6) - 0.4

    pyb_tools_utils.set_pose(plant_id, pyb_tools_utils.Pose(pyb_tools_utils.Point(x=px, y=py,
                                                                                  z=pyb_tools_utils.stable_z(plant_id, floor))))

def set_random_poses(plant_ids = [], floor = 0):

    for plant_id in plant_ids:
        set_random_pose(plant_id, floor)

def make_plant_responsive(plant_ids = []):

    for plant_id in plant_ids:
        p.setJointMotorControlArray(plant_id, [0, 1], p.VELOCITY_CONTROL, targetVelocities=[0, 0],
                                forces=[1e-1, 1e-1])  # make plant responsive to external force

def init_plant_ids(pids):

    global plant_ids

    plant_ids = pids


plants_ids = []

def step_sim():

    # plant_ids = [3, 4, 5, 6, 7]

    global plants_ids

    if(len(plants_ids) == 0):
        print("length of plant_ids must be at least 1")
        exit()

    for pid in plants_ids:
        plant_rot_joint_displacement_y, _, plant_hinge_y_reac, _ = pybullet.getJointState(pid, 0)
        plant_rot_joint_displacement_x, _, plant_hinge_x_reac, _ = pybullet.getJointState(pid, 1)
        pybullet.applyExternalTorque(pid, linkIndex=1,
                                     torqueObj=[-200 * plant_rot_joint_displacement_x,
                                                -200 * plant_rot_joint_displacement_y, 0],
                                     flags=pybullet.WORLD_FRAME)

    for t in range(200):
        pyb_tools_utils.step_simulation()
        # pyb_tools_utils.wait_for_duration(0.02)



def generate_tall_plants(num_plants, positions, floor):

    global plants_ids

    if(len(positions) != num_plants):
        print("Error! Make sure number of plants equals number of positions!")
        exit()

    # plants_ids = []
    plant_representations = []

    for i in range(num_plants):
        plants_ids.append(pyb_tools_utils.load_model('urdf/tall_plant_multi_dof2.urdf', fixed_base=True))
        set_random_pose(plants_ids[-1], floor, px = positions[i][0], py = positions[i][1])

        plant_representations.append(representation.TwoAngleRepresentation(plants_ids[-1], 1))


    make_plant_responsive(plants_ids)

    return plants_ids, plant_representations


envs = {
    "env0": [
        [0.4, 0.1],
        [0.05, -0.15],
        [-0.25, 0.60],
        [-0.25, 0.45],
        [-0.05, 0.50]
    ],
    "env1": [
        [0.4, 0.1],
        [0.12, -0.15],
        [-0.25, 0.60],
        [-0.25, 0.45],
        [-0.05, 0.50]
    ],
    "env2": [
        [0.15, -0.1],
        [-0.08, -0.15],
        [0.25, -0.60],
        [0.25, -0.43],
        [-0.05, -0.48]
    ],
    "env3": [
        [0.4, -0.25],
        [0.25, -0.25],
        [0.25, -0.60],
        [0.25, -0.40],
        [0.40, -0.60]
    ],
    "env4": [
        [0.15, -0.1],
        [-0.08, -0.15],
        [0.25, -0.60],
        [0.25, -0.35],
        [-0.05, -0.48]
    ],
    "env5": [
        [-0.05, -0.5],
        [0.15, -0.60],
        [0.25, -0.70],
        [0.25, -0.35],
        [0.40, -0.70]
    ],
    "env6": [
        [0.098, -0.10],
        [-0.005, -0.25],
        [0.1, -0.32],
        [0.14, -0.49],
        [0.4, -0.578]
    ],
    "env7": [
        [0.15, -0.10],
        [-0.10, -0.10],
        [0.15, -0.32],
        [-0.10, -0.32],
        [-0.1, -0.54],
        [0.15, -0.76],
        [-0.1, -0.76],
        [0.40, -0.10],
        [0.15, -0.54],
        [0.40, -0.32],
        [0.47, -0.54],
        [0.40, -0.76],
        [0.65, -0.10],
        [0.65, -0.32],
        [0.65, -0.54],
        [0.65, -0.76],
        [-0.35, -0.10],
        [-0.35, -0.32],
        # [-0.35, -0.54],
        # [-0.35, -0.76]
    ],
    "env8": [
    [0.15, -0.10],
    [0.15, -0.20],
    [-0.10, -0.10],
    [-0.10, -0.20],
    [0.15, -0.32],
    [-0.10, -0.32],
    [-0.10, -0.42],
    [-0.1, -0.54],
    [0.15, -0.76],
    [0.15, -0.66],
    [-0.1, -0.76],
    [-0.1, -0.66],
    [0.40, -0.10],
    [0.40, -0.20],
    [0.15, -0.54],
    [0.40, -0.32],
    [0.47, -0.54],
    [0.40, -0.76],
    [0.40, -0.66],
    [0.55, -0.10],
    [0.55, -0.20],
    [0.55, -0.32],
    [0.55, -0.42],
    [0.55, -0.54],
    [0.55, -0.76],
    [0.55, -0.66],
    [-0.25, -0.10],
    [-0.25, -0.20],
    [-0.25, -0.32],
    ]
}


def generate_plants(num_plants, positions, floor):

    # global plant_ids

    if(len(positions) != num_plants):
        print("Error! Make sure number of plants equals number of positions!")
        exit()

    plants_ids = []
    plant_representations = []

    for i in range(num_plants):
        plants_ids.append(pyb_tools_utils.load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True))
        set_random_pose(plants_ids[-1], floor, px = positions[i][0], py = positions[i][1])

        plant_representations.append(representation.TwoAngleRepresentation(plants_ids[-1], 1))


    make_plant_responsive(plants_ids)

    return plants_ids, plant_representations


# ENV5
# set_random_pose(plant1, floor, px = -0.05, py = -0.5)
# set_random_pose(plant2, floor, px = 0.15, py = -0.60)
# set_random_pose(plant3, floor, px = 0.25, py = -0.70)
# # set_random_pose(plant4, floor, px = 0.25, py = -0.33)
# set_random_pose(plant4, floor, px = 0.25, py = -0.35)
# set_random_pose(plant5, floor, px = 0.40, py = -0.70)

# envs = {
#     "env0": [
#         [0.4, 0.1],
#         [0.05, -0.15],
#         [-0.25, 0.60],
#         [-0.25, 0.45],
#         [-0.05, 0.50]
#     ],
#     "env1": [
#         [0.4, 0.1],
#         [0.12, -0.15],
#         [-0.25, 0.60],
#         [-0.25, 0.45],
#         [-0.05, 0.50]
#     ],
#     "env2": [
#         [0.15, -0.1],
#         [-0.08, -0.15],
#         [0.25, -0.60],
#         [0.25, -0.43],
#         [-0.05, -0.48]
#     ],
#     "env3": [
#         [0.4, -0.25],
#         [0.25, -0.25],
#         [0.25, -0.60],
#         [0.25, -0.40],
#         [0.40, -0.60]
#     ],
#     "env4": [
#         [0.15, -0.1],
#         [-0.08, -0.15],
#         [0.25, -0.60],
#         [0.25, -0.35],
#         [-0.05, -0.48]
#     ],
#     "env5": [
#         [-0.05, -0.5],
#         [0.15, -0.60],
#         [0.25, -0.70],
#         [0.25, -0.35],
#         [0.40, -0.70]
#     ]
# }