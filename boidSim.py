import numpy as np
import dcBoid
import vpython as vp

np.random.seed(0)

# Create a flock of boids
flock = dcBoid.BoidFlock(20, cohesion=0.01, alignment=3, separation=0.5)

# Simluation:

numSteps = 100000

boidObs = []

# Draw the edges of the cube
vp.curve(pos = [vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC1[2]), vp.vector(flock.cubeC2[0], flock.cubeC1[1], flock.cubeC1[2]), vp.vector(flock.cubeC2[0], flock.cubeC2[1], flock.cubeC1[2]), vp.vector(flock.cubeC1[0], flock.cubeC2[1], flock.cubeC1[2]), vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC1[2])], color = vp.color.blue)
vp.curve(pos = [vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC2[2]), vp.vector(flock.cubeC2[0], flock.cubeC1[1], flock.cubeC2[2]), vp.vector(flock.cubeC2[0], flock.cubeC2[1], flock.cubeC2[2]), vp.vector(flock.cubeC1[0], flock.cubeC2[1], flock.cubeC2[2]), vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC2[2])], color = vp.color.blue)
vp.curve(pos = [vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC1[2]), vp.vector(flock.cubeC1[0], flock.cubeC1[1], flock.cubeC2[2])], color = vp.color.blue)
vp.curve(pos = [vp.vector(flock.cubeC2[0], flock.cubeC1[1], flock.cubeC1[2]), vp.vector(flock.cubeC2[0], flock.cubeC1[1], flock.cubeC2[2])], color = vp.color.blue)
vp.curve(pos = [vp.vector(flock.cubeC2[0], flock.cubeC2[1], flock.cubeC1[2]), vp.vector(flock.cubeC2[0], flock.cubeC2[1], flock.cubeC2[2])], color = vp.color.blue)
vp.curve(pos = [vp.vector(flock.cubeC1[0], flock.cubeC2[1], flock.cubeC1[2]), vp.vector(flock.cubeC1[0], flock.cubeC2[1], flock.cubeC2[2])], color = vp.color.blue)

vp.scene.width = 1000
vp.scene.height = 1000
for i in range(numSteps):

    flock.stepBoid()

    for j in range(flock.numBoids):
        if len(boidObs) < flock.numBoids:
            boidObs.append(vp.sphere(pos = vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2]), radius = flock.boids[j].size, color = vp.color.red))
        else: 
            boidObs[j].pos = vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2])


print(flock.allPositions[0, :, :])
