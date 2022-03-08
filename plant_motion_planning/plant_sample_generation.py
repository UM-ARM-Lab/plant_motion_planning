from fileinput import close
import random

import numpy as np
import pybullet as p
import pybullet_data
import time
import cv2 
from skimage.metrics import structural_similarity as ssim
from plant_motion_planning.pybullet_tools.camera import PyBulletCamera
from plant_motion_planning.rand_gen_plants_programmatically_main import create_random_plant, create_plant_params
from plant_motion_planning.representation import TwoAngleRepresentation, TwoAngleRepresentation_mod, CharacterizePlant, \
    CharacterizePlant2


def img_to_sig(arr):
    """Convert a 2D array to a signature for cv2.EMD"""
    #https://samvankooten.net/2018/09/25/earth-movers-distance-in-python/
    # cv2.EMD requires single-precision, floating-point input
    sig = np.empty((arr.size, 3), dtype=np.float32)
    count = 0
    for i in range(arr.shape[0]):
        for j in range(arr.shape[1]):
            sig[count] = np.array([arr[i,j], i, j])
            count += 1
    return sig

if __name__ == "__main__":

    p.connect(p.DIRECT)#, options='--background_color_red=1.0 --background_color_green=0.0 --background_color_blue=0.0')
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)

    
    # wallc = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[1, 6, 3], collisionFramePosition=[0, 0, 0])

    # wallv = p.createVisualShape(
    #     p.GEOM_BOX, halfExtents=[0.1, 6, 3],
    #     visualFramePosition=[-4, 0, 0],
    #     rgbaColor = [1,0,1, 1]
    # )
    # inertiaShift = [0,0,-0.5]
    # p.createMultiBody(baseMass=1000,baseInertialFramePosition=inertiaShift,baseCollisionShapeIndex=wallc, baseVisualShapeIndex = wallv, basePosition = [0,0,1], useMaximalCoordinates=False)

    num_branches_per_stem = 0
    total_num_vert_stems = 2
    total_num_extensions = 1
    stem_half_length = 0.1
    stem_half_width = 0.1

    eye = np.array([2.0,0.0,2.0])
    lookat = np.array([0.0,0.0,0.0])
    up = np.array([-1.0,0.0,0.0])
    cam = PyBulletCamera(eye, lookat, camera_up=up)

    exec_times = []
    EMDs = []
    base_proximity = []
    link1_ori_dist = []
    link2_ori_dist = []
    



    base_points = {}
    main_stem_indices = []
    link_parent_indices = {}

    base_params, stems_params, main_stem_indices = create_plant_params(num_branches_per_stem, total_num_vert_stems,
                                                                    total_num_extensions, [0.0, 0.0])



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


    print("base base pose", base_pos)

    saved_base_pos = np.array(base_pos)
    print("base link2 ori", linkOrientations[2])
    saved_link2_ori = np.array(linkOrientations[2])
    print("base base ori", base_ori)
    saved_base_ori = np.array(base_ori)

    [img,depth,seg] = cam.get_image()

    compare_depth = depth
    seg = seg.astype(np.float32)
    seg[seg < 0] = 0
    compare_img = seg
    # depth_sig = img_to_sig(compare_depth*10)
    # compare_sig = img_to_sig(seg)
    # sig_depth = img_to_sig(depth)

    print(np.sum(seg-seg))
    print("max", np.max(seg))
    cv2.imshow('base',compare_img)
    cv2.waitKey(0)
    p.removeBody(base_id)

    most_overlap = 0

    best_params = [base_params, stems_params, main_stem_indices]

    all_params = []
    all_images = np.zeros((1000,64,64))


    for i in range (0,1000):
        x =  np.random.uniform(low=-0.2, high=0.2)
        y =  np.random.uniform(low=-0.2, high=0.2)
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

        time.sleep(1/60.0)
        p.stepSimulation()

        [img,depth,seg] = cam.get_image()
        seg[seg < 0] = 0
        all_images[i] = seg
        all_params.append([base_params, stems_params, main_stem_indices])
        p.removeBody(base_id)


    all_overlap = all_images + compare_img
    all_overlap[all_overlap<2] = 0
    sums = np.sum(np.sum(all_overlap, axis=1),axis=1)/2

    print("sums",sums)
    n_best = 10
    idx = (-sums).argsort()[:n_best]

    print(idx)

    print("best sums", sums.take(idx))

    test_img = all_images[idx[0]].astype(np.float32)

    cv2.imshow('new', test_img)
    cv2.waitKey(0)

    base_params, stems_params, main_stem_indices = best_params

    base_mass, col_base_id, vis_base_id, base_pos, base_ori = base_params
    link_Masses, linkCollisionShapeIndices, linkVisualShapeIndices, linkPositions, linkOrientations, \
        linkInertialFramePositions, linkInertialFrameOrientations, indices, jointTypes, axis = stems_params

    print("base pose",base_pos)
    print("base ori", base_ori)
    print("link2 ori", linkOrientations[2])

    basepos = np.array(base_pos)
    baseori = np.array(base_ori)
    link2ori = np.array(linkOrientations[2])

    print("base distance", np.linalg.norm(basepos-saved_base_pos,2))

    print("orientation distance 1", 1-np.abs(np.dot(baseori, saved_base_ori)))
    print("orientation distance 2", 1-np.abs(np.dot(link2ori, saved_link2_ori)))

    base_proximity.append(np.linalg.norm(basepos-saved_base_pos,2))
    link1_ori_dist.append(1-np.abs(np.dot(baseori, saved_base_ori)))
    link2_ori_dist.append(1-np.abs(np.dot(link2ori, saved_link2_ori)))


    p.removeBody(base_id)


    print(base_proximity)
    print(link1_ori_dist)
    print(link2_ori_dist)
