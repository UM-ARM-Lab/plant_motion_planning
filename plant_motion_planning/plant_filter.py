import numpy as np


class PlantPF:

    def __init__(self):
        self.particles = []
        self.weights = []

    def initialize_particles(self, image):
        # TODO initialize particles based on image

        # use plant_sample_generation to make this.
        pass

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
