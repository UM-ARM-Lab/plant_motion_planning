#l!/usr/bin/env python

"""
This program teleports the arm into collision with a plant and tests how many stepSim iterations it takes
until the plant settles. The purpose for this is to determine the number of sim steps needed for the 
environment to settle in the worst case.
"""

from __future__ import print_function
from audioop import mul

import random
import time

import numpy as np
import pybullet_data

from plant_motion_planning import cfg
from plant_motion_planning.env import SinglePlantEnv, MultiPlantWorld
from plant_motion_planning.planner import Planner
from plant_motion_planning.pybullet_tools.kuka_primitives import BodyConf, Command
from plant_motion_planning.pybullet_tools.utils import enable_gravity, connect, dump_world, set_pose, step_simulation, \
    draw_global_system, set_camera_pose, Pose, Point, Euler, BLOCK_URDF, load_model, disconnect, update_state, \
    disable_real_time, HideOutput, DRAKE_IIWA_URDF_EDIT, save_state, set_joint_positions, get_movable_joints, wait_for_duration
import pybullet as p
from plant_motion_planning.utils import load_plant_from_urdf

from matplotlib import pyplot as plt

import os
import argparse

def main():
    """
    Main function of this program.
    """

    # GUI connection client object
    # connect(use_gui=True,width=1000, height=700)
    # cli = connect(use_gui=True,width=1920, height=1080)
    cli = connect(use_gui=True,width=1000, height=700)
    disable_real_time()

    # Set camera pose to desired position and orientation
    set_camera_pose((2.5, -1.06, 3.5), (2.5, 2.5, 0.0))

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

    enable_gravity()

    # Draw X, Y, Z axes
    draw_global_system()

    # Base position in X and Y
    plant_pos_xy_limits = ((0, 0.35), (0.15, 0.50))
    # initialize deflection limit from arguments input
    deflection_limit = 2.0

    # Base offset X and Y
    base_offset_xs = [0]
    base_offset_ys = [0]

    # Generate random plant for each world
    multi_world_env = MultiPlantWorld(base_offset_xs, base_offset_ys, deflection_limit, loadPath='models/',physicsClientId=cli)
    plant_id = multi_world_env.envs[(0, 0)].plant_id
    joint_list = multi_world_env.envs[(0, 0)].joint_list

    # for i in range(len(joint_list)):
    #     link_pose = p.getLinkState(plant_id, i)[0]
    #     p.addUserDebugText(str(i), link_pose, textSize=0.5)
    #     print(link_pose)

    num_iterations = 400
    link_idx = 7
    step_num = []
    delta_link_pose = []

    prev_pos = p.getLinkState(plant_id, link_idx)[0]
    for t in range(num_iterations):
            for xy, env in multi_world_env.envs.items():
                env._restore_plant_joints()
            step_simulation()

            step_num.append(t)

            curr_pos = p.getLinkState(plant_id, link_idx)[0]
            delta_link_pose.append(np.linalg.norm(np.array(curr_pos) - np.array(prev_pos)))
            prev_pos = curr_pos

    input("Press enter to quit")
    print('Quitting simulation')
    disconnect()
    
    
    plt.title('Sim step effect on link in collision')
    plt.plot(step_num, delta_link_pose)
    plt.xlabel('Simulation step #')
    plt.ylabel('Change in link position')
    plt.show()

if __name__ == '__main__':
    main()