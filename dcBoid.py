import numpy as np


class BoidFlock:

    def __init__(self, num_boids, cohesion: 'np.ndarray' = False, alignment = False, separation = False):
        self.boids = []

        simStepsPerMemAlloc = 1000
        self.numDim = 3

        self.cubeC1 = np.array([0,0,0])
        self.cubeC2 = np.array([100,100,100])

        # Boids will be initialized with these traits
        self.cohesion = cohesion
        self.alignment = alignment
        self.separation = separation

        # Initialize the array to store all the positions of the boids
        self.allPositions = np.full((simStepsPerMemAlloc, num_boids, self.numDim), np.nan)

        # Initialize the array to store all the velocities of the boids
        self.allVelocities = np.full((simStepsPerMemAlloc, num_boids, self.numDim), np.nan)

        # Rows are time steps, columns are boids, and depth is x, y, z

        # Add all the boids to the flock
        for i in range(num_boids):
            boidPosition = np.random.uniform(self.cubeC1, self.cubeC2)
            self.allPositions[0, i, :] = boidPosition

            boidVelocity = np.random.uniform(0, 1, self.numDim)
            self.allVelocities[0, i, :] = boidVelocity

            self.boids.append(Boid(boidPosition, boidVelocity, i))

        self.boidsInit = True
        self.numBoids = num_boids

        


class Boid:

    def __init__(self, position: 'np.ndarray', velocity: 'np.ndarray', boidID):
        self.position = position
        self.velocity = velocity
        self.boidID = boidID