import numpy as np
import pybullet
import pybullet as p
# from pybullet_tools.utils import set_pose, Pose, Point, stable_z, step_simulation
from pybullet_utils import urdfEditor

import plant_motion_planning.pybullet_tools.utils as pyb_tools_utils
from plant_motion_planning.rand_gen_plants_programmatically_main import create_random_plant
from plant_motion_planning import representation
from plant_motion_planning.representation import CharacterizePlant, CharacterizePlant2


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
joint_list = -1
base_id = -1

def step_sim():

    # plant_ids = [3, 4, 5, 6, 7]

    global plants_ids

    # a;ld

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


def step_sim_v2():

    # plant_ids = [3, 4, 5, 6, 7]

    global base_id, joint_list

    kp = 3000

    # print("base id: ", base_id)
    # print("joint_list: ", joint_list)
    # input("chk pt 2")

    for i, joint_idx in enumerate(range(len(joint_list) - 1, 0, -2)):
        plant_rot_joint_displacement_y, _, plant_hinge_x_reac, _ = p.getJointState(base_id, joint_idx)
        plant_rot_joint_displacement_x, _, plant_hinge_y_reac, _ = p.getJointState(base_id, joint_idx - 1)

        p.applyExternalTorque(base_id, linkIndex=joint_idx,
                              torqueObj=[-kp * plant_rot_joint_displacement_x,
                                         -kp * plant_rot_joint_displacement_y, 0],
                              flags=p.LINK_FRAME)

    for t in range(200):
        pyb_tools_utils.step_simulation()


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

    global plant_ids

    if(len(positions) != num_plants):
        print("Error! Make sure number of plants equals number of positions!")
        exit()

    # plants_ids = []
    plant_representations = []

    for i in range(num_plants):
        plants_ids.append(pyb_tools_utils.load_model('urdf/short_plant_multi_dof2.urdf', fixed_base=True))
        set_random_pose(plants_ids[-1], floor, px = positions[i][0], py = positions[i][1])

        plant_representations.append(representation.TwoAngleRepresentation(plants_ids[-1], 1))


    make_plant_responsive(plants_ids)

    return plants_ids, plant_representations


def generate_random_plant(num_branches_per_stem, total_num_vert_stems, total_num_extensions, base_pos_xy, stem_half_length=0.1,
                        stem_half_width=0.1, physicsClientId=None, save_plant=False):

    global joint_list, base_id

    # Create plant
    base_id, joint_list, main_stem_indices, base_points, \
    base_pos, link_parent_indices = create_random_plant(num_branches_per_stem, total_num_vert_stems,
                                                                              total_num_extensions, base_pos_xy)

    # Make plant responsive
    p.setJointMotorControlArray(base_id, joint_list, p.VELOCITY_CONTROL, targetVelocities=len(joint_list) * [0],
                                forces=len(joint_list) * [1e-1])  # make plant responsive to external force

    if (save_plant):
        urdf_parser = urdfEditor.UrdfEditor()
        urdf_parser.initializeFromBulletBody(base_id, physicsClientId=physicsClientId)
        urdf_parser.saveUrdf("plant.urdf")

        plant_params = [joint_list, main_stem_indices, base_points, base_pos]

        import pickle
        with open('plant_params.pkl', 'wb') as fp:
            pickle.dump(plant_params, fp)

    # Characterize plant
    base_rep = CharacterizePlant2(base_id, base_points, main_stem_indices, link_parent_indices)

    return base_id, base_rep, joint_list, base_id


def load_plant_from_urdf(plant_urdf_name=None, plant_params_name=None, base_pos_offset_xy=None):

    global joint_list, base_id

    # Create plant
    # base_id, joint_list, main_stem_indices, base_points = create_random_plant(num_branches_per_stem,
    #                                                                           total_num_vert_stems,
    #                                                                           total_num_extensions)

    base_id = p.loadURDF(plant_urdf_name, useFixedBase=True)
    for j in range(-1, p.getNumJoints(base_id)):
        p.changeDynamics(base_id, j, jointLowerLimit=-1.5, jointUpperLimit=1.5, jointDamping=10, linearDamping=1.5)

    import pickle
    with open(plant_params_name, 'rb') as fp:
        plant_params = pickle.load(fp)

    joint_list, main_stem_indices, base_points, base_pos = plant_params

    # print(joint_list)

    pyb_tools_utils.set_pose(base_id, pyb_tools_utils.Pose(
        pyb_tools_utils.Point(x=base_pos[0] + base_pos_offset_xy[0], y=base_pos[1] + base_pos_offset_xy[1],
                              z=base_pos[2])))

    # Make plant responsive
    p.setJointMotorControlArray(base_id, joint_list, p.VELOCITY_CONTROL, targetVelocities=len(joint_list) * [0],
                                forces=len(joint_list) * [1e-1])  # make plant responsive to external force

    # Offset base points
    for base_points_key in base_points:
        base_points[base_points_key][0] += base_pos_offset_xy[0]
        base_points[base_points_key][1] += base_pos_offset_xy[1]

    # Characterize plant
    base_rep = CharacterizePlant(base_id, base_points, main_stem_indices)

    return base_id, base_rep, joint_list, base_id