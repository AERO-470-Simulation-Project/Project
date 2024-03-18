import numpy as np


class BoidFlock:

    def __init__(self, num_boids, cohesion: 'np.ndarray' = False, alignment = False, separation = False):
        self.boids = []

        # Boids will be initialized with these traits
        self.cohesion = cohesion
        self.alignment = alignment
        self.separation = separation
        



class Boid:

    def __init__(self, position, velocity):
        self.postion = position
        self.velocity = velocity