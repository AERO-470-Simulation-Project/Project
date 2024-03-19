import numpy as np
import dcBoid
import vpython as vp

# Create a flock of boids
flock = dcBoid.BoidFlock(3, 0.2, 0.2, 0.2)

# Simluation:

numSteps = 100000

boidObs = []

for i in range(numSteps):

    print(flock.allPositions[i, :, :])
    flock.stepBoid()

    for j in range(flock.numBoids):
        if len(boidObs) < flock.numBoids:
            boidObs.append(vp.sphere(pos = vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2]), radius = flock.boids[j].size, color = vp.color.red))
        else: 
            boidObs[j].pos = vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2])

    vp.rate(60)

print(flock.allPositions[0, :, :])
