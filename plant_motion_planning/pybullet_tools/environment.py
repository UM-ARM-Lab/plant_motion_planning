import pybullet as p
import os
import torch
from plant_motion_planning.plant import Plant
from plant_motion_planning.pybullet_tools.utils import HDT_MICHIGAN_URDF
from plant_motion_planning import cfg

# Garden dimensions in meters
GARDEN_HEIGHT = 1.2192
GARDEN_LENGTH = 2.4384
GARDEN_BORDER = 0.1

LONG_WALL_URDF = os.path.join(cfg.ROOT_DIR, 'models/wall_long.urdf')
SHORT_WALL_URDF = os.path.join(cfg.ROOT_DIR, 'models/wall_short.urdf')

def find_z_coord():
    # Find z-coord for robot
    robot = p.loadURDF(HDT_MICHIGAN_URDF, basePosition=(0, 0, 5))
    for t in range(1000):
        p.stepSimulation()
    pose, quat = p.getBasePositionAndOrientation(robot)
    z_coord = pose[2]
    p.removeBody(robot)
    return z_coord

class Environment:
    def __init__(self, offset, num_plants, z_coord, num_steps, device, base_position_border=2):
        self.plants = []
        self.x_offset, self.y_offset = offset
        self.num_plants_x, self.num_plants_y = num_plants
        self.num_steps = num_steps
        self.device = device

        self.min_limits = torch.tensor([-base_position_border, -base_position_border], device=self.device)
        self.max_limits = torch.tensor([GARDEN_LENGTH + base_position_border, GARDEN_HEIGHT + base_position_border], device=self.device)

        # Load robot
        self.robot = p.loadURDF(HDT_MICHIGAN_URDF, basePosition=(-0.5 + self.x_offset, -0.5 + self.y_offset, z_coord), useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
        for j in range(p.getNumJoints(self.robot)):
            p.changeDynamics(self.robot, j, mass=9999999999)


        # Setup garden walls
        wall1 = p.loadURDF(SHORT_WALL_URDF, useFixedBase=True, basePosition=(0 + self.x_offset, GARDEN_HEIGHT / 2 + self.y_offset, 0))
        wall2 = p.loadURDF(SHORT_WALL_URDF, useFixedBase=True, basePosition=(GARDEN_LENGTH + GARDEN_BORDER + self.x_offset, GARDEN_HEIGHT / 2 + self.y_offset, 0))
        wall3 = p.loadURDF(LONG_WALL_URDF, useFixedBase=True, basePosition=(GARDEN_HEIGHT + self.x_offset, 0 + self.y_offset, 0))
        wall4 = p.loadURDF(LONG_WALL_URDF, useFixedBase=True, basePosition=(GARDEN_HEIGHT + GARDEN_BORDER + self.x_offset, GARDEN_HEIGHT + self.y_offset, 0))

        # Create plants
        num_branches_per_stem = 1
        total_num_vert_stems = 2
        total_num_extensions = 1

        for y in range(self.num_plants_y):
            y_pos = (y + 1) * GARDEN_HEIGHT / (self.num_plants_y + 1) + self.y_offset
            for x in range(self.num_plants_x):
                x_pos = (x + 1) * GARDEN_LENGTH / (self.num_plants_x + 1) + self.x_offset
                plant = Plant(num_branches_per_stem, total_num_vert_stems, total_num_extensions, (x_pos, y_pos))
                self.plants.append(plant)
    
    def sample_base_position(self):
        while True:
            sample = (self.max_limits - self.min_limits) * torch.rand(2, device=self.device) + self.min_limits
            if self.is_sample_valid(sample):
                return sample

    def is_sample_valid(self, sample):
        # Check if sampled pose is inside garden
        return not ((sample[0] > 0) and (sample[0] < GARDEN_LENGTH) and (sample[1] > 0) and (sample[1] < GARDEN_HEIGHT))

    def save_state(self):
        return p.saveState()
    def restore_state(self, state_id):
        p.restoreState(stateId=state_id)

    def step_env(self):
        for i in range(self.num_steps):
            for plant in self.plants:
                plant.apply_restoring_force()
            p.stepSimulation()

    def check_deflection(self):
        for plant in self.plants:
            if plant.check_deflection():
                print("Deflection limit exceeded")
                return True
        return False
        
