import numpy as np
import matplotlib.pyplot as plt

class Boid:
    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity

    def update(self, flock):
        # Calculate the average position and velocity of neighboring boids
        avg_position = np.mean([b.position for b in flock.boids], axis=0)
        avg_velocity = np.mean([b.velocity for b in flock.boids], axis=0)

        # Update the boid's velocity based on the average position and velocity
        self.velocity += (avg_position - self.position) * 0.01 + avg_velocity * 0.1

        # Update the boid's position based on its velocity
        self.position += self.velocity

class Flock:
    def __init__(self, num_boids):
        self.boids = []
        for _ in range(num_boids):
            position = np.random.rand(2) * 10  # Random initial position
            velocity = np.random.rand(2) - 0.5  # Random initial velocity
            boid = Boid(position, velocity)
            self.boids.append(boid)

    def update(self):
        for boid in self.boids:
            boid.update(self)

    def plot(self):
        positions = np.array([boid.position for boid in self.boids])
        plt.scatter(positions[:, 0], positions[:, 1])
        plt.show()

# Example usage
flock = Flock(num_boids=50)

stop_simulation = False
while not stop_simulation:
    flock.update()
    flock.plot()

    # Check for user input to stop the simulation
    user_input = input("Press 's' to stop the simulation: ")
    if user_input.lower() == 's':
        stop_simulation = True
