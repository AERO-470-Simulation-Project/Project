import numpy as np
import dcBoid
import vpython as vp

np.random.seed(0)

# Create a flock of boids
flock = dcBoid.BoidFlock(5, 20, cohesion=0.01, alignment=3, separation=0.5)

# Simluation:

numSteps = 100000

boidObs = []
hawkObs = []

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
    flock.stepBoid()  # Update positions and velocities

    for hawk in flock.hawks:
        caught_boid_index = hawk.killBoid(flock)
        if caught_boid_index != -1:
            flock.removeBoid(caught_boid_index)

    # Update boid visualization
    for j in range(flock.numBoids):
        if i == 0:  # Initialize spheres for boids
            boidObs.append(vp.sphere(pos=vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2]), radius=flock.boids[j].size, color=vp.color.red))
        else:
            boidObs[j].pos = vp.vector(flock.allPositions[i, j, 0], flock.allPositions[i, j, 1], flock.allPositions[i, j, 2])
    
    # Update hawk visualization
    for k in range(flock.numHawks):
        if i == 0:  # Initialize spheres for hawks
            hawkObs.append(vp.sphere(pos=vp.vector(flock.allHawkPositions[i, k, 0], flock.allHawkPositions[i, k, 1], flock.allHawkPositions[i, k, 2]), radius=flock.hawks[k].size, color=vp.color.orange))
        else:
            hawkObs[k].pos = vp.vector(flock.allHawkPositions[i, k, 0], flock.allHawkPositions[i, k, 1], flock.allHawkPositions[i, k, 2])
    


print(flock.allPositions[0, :, :])
print(flock.allHawkPositions[-1, :, :])
