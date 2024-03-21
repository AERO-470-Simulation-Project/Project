import unittest
import numpy as np
from dcBoid import Boid, HawkBoid, BoidFlock

class TestBoidSimulation(unittest.TestCase):
    def setUp(self):
        # Setup for each test method
        self.num_boids = 10
        self.num_hawks = 2
        self.flock = BoidFlock(self.num_hawks, self.num_boids, cohesion=0.01, alignment=0.5, separation=1.5)

    def test_boid_initialization(self):
        # Test that boids are initialized correctly
        self.assertEqual(len(self.flock.boids), self.num_boids)
        for boid in self.flock.boids:
            self.assertTrue(np.all(boid.position >= 0))
            self.assertTrue(np.all(boid.position <= 100))
            self.assertIsNotNone(boid.velocity)

    def test_hawk_initialization(self):
        # Test that hawks are initialized correctly
        self.assertEqual(len(self.flock.hawks), self.num_hawks)
        for hawk in self.flock.hawks:
            self.assertTrue(np.all(hawk.position >= 0))
            self.assertTrue(np.all(hawk.position <= 100))
            self.assertIsNotNone(hawk.velocity)

    def test_boid_movement(self):
        # Test that boids move as expected
        initial_positions = np.array([boid.position for boid in self.flock.boids])
        self.flock.stepBoid()
        for idx, boid in enumerate(self.flock.boids):
            self.assertFalse(np.array_equal(boid.position, initial_positions[idx]))

    def test_hawk_hunting(self):
        # Test that hawks hunt boids (simplified test assuming hawks can catch a boid immediately if within range)
        boid = Boid(np.array([50, 50, 50]), np.array([0, 1, 0]), 0, 0.01, 0.5, 1.5)
        hawk = HawkBoid(np.array([52, 50, 50]), np.array([0, -1, 0]), 0)
        self.flock.boids = [boid]
        self.flock.hawks = [hawk]
        prey_index = hawk.detectPrey(self.flock)
        self.assertEqual(prey_index, 0)  # Assuming detectPrey returns the index of the boid

if __name__ == '__main__':
    unittest.main()
