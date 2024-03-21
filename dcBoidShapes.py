import vpython as vp

import numpy as np

class BoidShape:
    def __init__(self, pos: 'np.ndarray', unitVec: 'np.ndarray', size = 3, color: 'vp.color' = vp.color.blue):
        self.pos = vp.vector(pos[0], pos[1], pos[2])
        self.unitVec = vp.vector(unitVec[0] * size, unitVec[1] * size, unitVec[2] * size)
        self.size = size
        self.color = color
        self.boid = vp.cone(pos=self.pos, axis=self.unitVec, radius = size/3, color=self.color)

    def moveBoid(self, pos: 'np.ndarray', unitVec: 'np.ndarray'):
        self.pos = vp.vector(pos[0], pos[1], pos[2])
        self.unitVec = vp.vector(unitVec[0] * self.size, unitVec[1] * self.size, unitVec[2] * self.size)
        self.boid.pos = self.pos
        self.boid.axis = self.unitVec

