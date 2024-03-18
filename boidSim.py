import numpy as np
import dcBoid


# Create a flock of boids
flock = dcBoid.BoidFlock(3, True, True, True)

print(flock.boids[0].position)
print(flock.allPositions[0, 0, :])
print(flock.boids[1].position)
print(flock.allPositions[0, 1, :])
print(flock.boids[2].position)
print(flock.allPositions[0, 2, :])


