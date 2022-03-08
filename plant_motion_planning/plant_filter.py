from fileinput import close
import random

from plant_motion_planning.val import  *
import numpy as np
import pybullet as p
import pybullet_data
import time
import cv2
from skimage.metrics import structural_similarity as ssim
from plant_motion_planning.pybullet_tools.camera import PyBulletCamera
from plant_motion_planning.utils import generate_random_plant
from plant_motion_planning.rand_gen_plants_programmatically_main import create_random_plant, create_plant_params, create_plant_params2
from plant_motion_planning.representation import TwoAngleRepresentation, TwoAngleRepresentation_mod, CharacterizePlant, \
    CharacterizePlant2
from plant_motion_planning.env import SinglePlantOnlyEnv, MultiPlantOnlyWorld, SingleMultiPlantOnlyEnvironment, MultiMultiPlantOnlyWorld
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, update_state, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT, save_state, set_joint_positions, get_movable_joints, pairwise_collision, step_simulation, inverse_kinematics

class PlantPF:

    def __init__(self):
        self.particles = []
        self.weights = []




    def initialize_particles(self, image):
        # TODO initialize particles based on image
        # use plant_sample_generation to make this.

        for i in range(0,4):
            particle, weight = get_n_samples(10, 100, seg)
            self.particles.append(particle)
            self.weights.append(weight)

        # Weights should sum up to 1, so normalize based on the sum of the weights
        self.weights = np.array(self.weights)
        self.weights = self.weights / self.weights.sum()

        print("weights of particles", self.weights)

        return self.particles

    def select_action(self):
        # figure out what the most uncertain plant is somehow and move that plant using an action of
        # the robots arms in the vicinity of that plant. prior to actually using the arm, I just move a
        # horizontal rod across the plant.

        pass

    def update_filter(self, measurement, environment):
        # First resample the posterior (pruning step) based on the particle weights
        # SELECT ACTION.
        # Second, apply the action to each particle.
        # Third, evaluate each particle based on the image they would generate to get new weights
        # Fourth, estimate the true state of the environment - likely just weighted average over the parameters.
        # Fifth actually perform the action in the real environment and get the new image.

        # TODO step the filter after selecting an action



        pass

    def apply_dynamics(self):
        # TODO execute action in simulator
        pass

    def filter_predict(self, chosen_action):
        # TODO Apply action to each particle
        pass


# Function to measure distance between to images, to be swapped to appropriate metric eventually,
# for now it is just a pixel counting method.
def image_distance(im1, im2):
    pass


def resample(particles):
    # TODO resample particles
    pass


def get_n_samples(n_best, num_trials, compare_img):
    all_params = []
    all_images = np.zeros((num_trials, 32, 32))
    num_branches_per_stem = 0
    total_num_vert_stems = 2
    total_num_extensions = 1
    save_compare = compare_img.copy()
    #Each particle is composed of 3 plants (for now, to be increased)
    #The weight of the particle is percentage overlap.


    for i in range(0, num_trials):
        x = np.random.uniform(low=-0.3, high=0.3)
        y = np.random.uniform(low=-0.3, high=0.3)+8


        base_params, stems_params, main_stem_indices, base_points, link_parent_indices = create_plant_params2(num_branches_per_stem, total_num_vert_stems,
                                                                           total_num_extensions, [x, y])
        # print("base points during creation", base_points)
        base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
        link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
        linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params

        # Create plant to just take a picture of it, can then reuse the parameters to restore the plant.



        base_id = p.createMultiBody(
            base_mass, col_base_id, vis_base_id, base_pos, base_ori,
            linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
            linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
            linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
            linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
            linkJointTypes=jointTypes, linkJointAxis=axis
        )

        time.sleep(1 / 60.0)
        # p.stepSimulation()

        [img, depth, seg] = cam.get_image()
        seg[seg < 0] = 0
        seg[seg > 1] = 1
        all_images[i] = seg
        all_params.append([base_params, stems_params, main_stem_indices, base_points, link_parent_indices])
        p.removeBody(base_id)

    best_three = []

    all_overlap = all_images + compare_img
    all_overlap[all_overlap < 2] = 0
    sums = np.sum(np.sum(all_overlap, axis=1), axis=1) / 2

    idx = np.argmax(sums)
    best_three.append(all_params[idx])

    base_params, stems_params, main_stem_indices, base_points, link_parent_indices = all_params[idx]
    base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params

    # print("base pos plant 1", base_pos)
    # print("base points plant 1", base_points)

    test_img = all_images[idx].astype(np.float32)
    # cv2.imshow('new', test_img)

    #Re-run the overlap after 'deleting' the best plant from the comparison image.
    compare_img = compare_img - test_img
    compare_img[compare_img < 0] = 0
    all_overlap = all_images + compare_img
    all_overlap[all_overlap < 2] = 0
    sums = np.sum(np.sum(all_overlap, axis=1), axis=1) / 2
    idx = np.argmax(sums)
    best_three.append(all_params[idx])
    base_params, stems_params, main_stem_indices, base_points, link_parent_indices = all_params[idx]
    base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
    # print("base pos plant 2", base_pos)
    # print("base points plant 2", base_points)

    test_img = all_images[idx].astype(np.float32)
    # cv2.imshow('new2', compare_img)


    compare_img = compare_img - test_img
    compare_img[compare_img < 0] = 0
    all_overlap = all_images + compare_img
    all_overlap[all_overlap < 2] = 0
    sums = np.sum(np.sum(all_overlap, axis=1), axis=1) / 2
    idx = np.argmax(sums)
    best_three.append(all_params[idx])


    test_img = all_images[idx].astype(np.float32)
    # cv2.imshow('new3', compare_img)
    compare_img = compare_img - test_img
    compare_img[compare_img < 0] = 0
    # cv2.waitKey(0)

    print("Number of pixels in original", save_compare.sum())
    print("Number of pixels remaining", compare_img.sum())

    weight = save_compare.sum()-compare_img.sum()

    return best_three, weight

def generate_plant(x,y):

    num_branches_per_stem = 0
    total_num_vert_stems = 2
    total_num_extensions = 1

    base_params, stems_params, main_stem_indices = create_plant_params(num_branches_per_stem, total_num_vert_stems,
                                                                    total_num_extensions, [x, y])

    base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
    link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
        linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params


    base_id = p.createMultiBody(
        base_mass, col_base_id, vis_base_id, base_pos, base_ori,
        linkMasses=link_Masses, linkCollisionShapeIndices=linkCollisionShapeIndices,
        linkVisualShapeIndices=linkVisualShapeIndices, linkPositions=linkPositions,
        linkOrientations=linkOrientations, linkInertialFramePositions=linkInertialFramePositions,
        linkInertialFrameOrientations=linkInertialFrameOrientations, linkParentIndices=indices,
        linkJointTypes=jointTypes, linkJointAxis=axis
    )
    return base_params, stems_params, main_stem_indices, base_id


right_arm_joints = [
    'joint1',
    'joint2',
    'joint3',
    'joint4',
    'joint5',
    'joint6',
    'joint7',
]

left_arm_joints = [
    'joint41',
    'joint42',
    'joint43',
    'joint44',
    'joint45',
    'joint46',
    'joint47',
]

if __name__ == "__main__":
    cli = connect(use_gui=True,width=1000, height=700)#, options='--background_color_red=1.0 --background_color_green=0.0 --background_color_blue=0.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)
    disable_real_time()
    enable_gravity()

    start_orientation = [0, 0, 0]
    start_pos = [0, 0, 0.2]

    p.setAdditionalSearchPath(pybullet_data.getDataPath())


    # robot = Val(start_pos,start_orientation)

    num_branches_per_stem = 0
    total_num_vert_stems = 2
    total_num_extensions = 0
    deflection_limit = 2.0
    base_offset_xs = [0,2]
    base_offset_ys = [0,2]
    plant_pos_xy_limits = ((-0.3, 0.3), (-0.3, 0.3))
    num_plants = 3
    # multi_world_env = MultiPlantOnlyWorld(base_offset_xs, base_offset_ys, deflection_limit, num_branches_per_stem, total_num_vert_stems,
    #                                   total_num_extensions, plant_pos_xy_limits, physicsClientId=cli)

    # single_plant_env = SingleMultiPlantOnlyEnvironment(deflection_limit, num_branches_per_stem, total_num_vert_stems,
    #                                    total_num_extensions, plant_pos_xy_limits, base_offset_xy=(-2, 0 ), physicsClientId=cli, num_plants=num_plants)
    np.random.seed(3)
    random.seed(3)
    # multi_world_env = MultiMultiPlantOnlyWorld(base_offset_xs, base_offset_ys, deflection_limit, num_branches_per_stem, total_num_vert_stems,
    #                                    total_num_extensions, plant_pos_xy_limits,  physicsClientId=cli, num_plants=num_plants)





    single_plant_env = SingleMultiPlantOnlyEnvironment(deflection_limit, num_branches_per_stem, total_num_vert_stems,
                                       total_num_extensions, plant_pos_xy_limits, base_offset_xy=(0, -2.0 ), physicsClientId=cli, num_plants=num_plants)

    # Generate 3 plants to begin with
    # base_params1, stems_params1, main_stem_indices1, base_id1 = generate_plant(0.0,2.0)
    # base_params2, stems_params2, main_stem_indices2, base_id2 = generate_plant(0.05, 2.05)
    # base_params3, stems_params3, main_stem_indices3, base_id3 = generate_plant(-0.15,1.85)

    # urdf = robot.urdf
    # print(robot.get_eef_pos("left"))
    # eepose = robot.get_eef_pos("left")
    # print(inverse_kinematics(urdf, "left_tool_joint", eepose))

    # Settle the environment
    t = 0
    while t < 240:
        t = t + 1
        #No action to take
        single_plant_env.step(None)
        # multi_world_env.step_no_action()
        step_simulation()
        target = [0.5, 0.20, 0.8 ,  0,0,0]


        time.sleep(1/240.0)


    eye = np.array([1.0,8.0,1.5])
    lookat = np.array([0.0,8.0,0.5])
    eye2 = np.array([1.0,-2.0,1.5])
    lookat2 = np.array([0.0,-2.0,0.5])

    up = np.array([-1.0,0.0,0.0])
    cam = PyBulletCamera(eye, lookat, image_dim=(32, 32), camera_up=up)
    cam2 = PyBulletCamera(eye2, lookat2, image_dim=(32, 32), camera_up=up)
    #
    [img, depth, seg] = cam2.get_image()
    seg = seg.astype(np.float32)
    seg[seg < 0] = 0
    seg[seg > 1] = 1
    cv2.imshow('original', seg)
    cv2.waitKey(0)


    #
    # # p.removeBody(base_id1)
    # # p.removeBody(base_id2)
    # # p.removeBody(base_id3)
    #
    #
    # print("Image:", seg.sum())
    #best_three, weights = get_n_samples(10, 100, seg)
    # plants = [best_three, best_three, best_three, best_three]




    # Actual filtering stuff
    pf = PlantPF()
    plants = pf.initialize_particles(seg)




    multi_world_env = MultiMultiPlantOnlyWorld(base_offset_xs, base_offset_ys, deflection_limit, num_branches_per_stem, total_num_vert_stems,
                                       total_num_extensions, plant_pos_xy_limits,  physicsClientId=cli, num_plants=num_plants, plant_sets=plants)

    cameras = {}
    for x in base_offset_xs:
        for y in base_offset_ys:
            e = np.array([1.0+x, 0+y, 1.5])
            l = np.array([0.0+x, 0+y, 0.5])
            cameras[(x,y)] = PyBulletCamera(e, l, image_dim=(32, 32), camera_up=up)




    t = 0
    while t < 1000:
        t = t + 1
        #No action to take
        single_plant_env.step(None)
        multi_world_env.step_no_action()
        step_simulation()
        time.sleep(1/240.0)

    for x in base_offset_xs:
        for y in base_offset_ys:
            c = cameras[(x,y)]
            [img, depth, seg] = c.get_image()
            seg = seg.astype(np.float32)
            seg[seg < 0] = 0
            seg[seg > 1] = 1
            cv2.imshow(str((x,y)), seg)

    cv2.waitKey(0)


    #
