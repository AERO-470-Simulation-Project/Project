import numpy as np


class BoidFlock:

    def __init__(self, num_hawks, num_boids, cohesion = 0, alignment = 0, separation = 0, speed = 1, dt = 1/6):
        self.boids = []
        self.hawks = []
        self.step = 0
        self.dt = dt

        self.simStepsPerMemAlloc = 1000
        self.numDim = 3

        self.cubeC1 = np.array([0,0,0])
        self.cubeC2 = np.array([100,100,100])

        # Boids will be initialized with these traits
        self.cohesion = cohesion
        self.alignment = alignment
        self.separation = separation

        # Initialize the array to store all the positions of the boids
        self.allPositions = np.full((self.simStepsPerMemAlloc, num_boids, self.numDim), np.nan)

        # Initialize the array to store all the velocities of the boids
        self.allUnitVeloc = np.full((self.simStepsPerMemAlloc, num_boids, self.numDim), np.nan)

        # Initialize array for Hawks Position and Velocity
        self.allHawkPositions = np.full((self.simStepsPerMemAlloc, num_hawks, self.numDim), np.nan)
        self.allHawkVelocities = np.full((self.simStepsPerMemAlloc, num_hawks, self.numDim), np.nan)

        # Rows are time steps, columns are boids, and depth is x, y, z

        # Add all the boids to the flock
        for i in range(num_boids):
            boidPosition = np.random.uniform(self.cubeC1, self.cubeC2)
            self.allPositions[0, i, :] = boidPosition

            boidVelocity = np.random.uniform(0, 1, self.numDim)
            self.allUnitVeloc[0, i, :] = boidVelocity/np.linalg.norm(boidVelocity)

            self.boids.append(Boid(boidPosition, boidVelocity, i, self.cohesion, self.alignment, self.separation, speed = speed))

        self.numBoids = num_boids

        for i in range(num_hawks):
            hawkPosition = np.random.uniform(self.cubeC1, self.cubeC2)
            self.allHawkPositions[0, i, :] = hawkPosition

            hawkVelocity = np.random.uniform(0, 1, self.numDim)
            self.allHawkVelocities[0, i, :] = hawkVelocity/np.linalg.norm(hawkVelocity)

            self.hawks.append(HawkBoid(hawkPosition, hawkVelocity, i))

        self.numHawks = num_hawks

    def stepBoid(self):

        # Check if the arrays need to be expanded
        if self.step == self.allPositions.shape[0] - 1:
            self.allPositions = np.append(self.allPositions, np.full((self.simStepsPerMemAlloc, self.numBoids, self.numDim), np.nan), axis = 0)
            self.allUnitVeloc = np.append(self.allUnitVeloc, np.full((self.simStepsPerMemAlloc, self.numBoids, self.numDim), np.nan), axis = 0)      

        if self.step == self.allHawkPositions.shape[0] - 1:
            self.allHawkPositions = np.append(self.allHawkPositions, np.full((self.simStepsPerMemAlloc, self.numHawks, self.numDim), np.nan), axis = 0)
            self.allHawkVelocities = np.append(self.allHawkVelocities, np.full((self.simStepsPerMemAlloc, self.numHawks, self.numDim), np.nan), axis = 0)  
        
        # Move all the boids
        for i in range(self.numBoids):
            self.allPositions[self.step + 1, i, :], self.allUnitVeloc[self.step + 1, i, :] = self.boids[i].movement(self)

        # Move all the hawks
        for i in range(self.numHawks):
            self.allHawkPositions[self.step + 1, i, :], self.allHawkVelocities[self.step + 1, i, :] = self.hawks[i].movement(self)

        # Update boid states
        for i in range(self.numBoids):
            self.boids[i].position = self.allPositions[self.step + 1, i, :]
            self.boids[i].velocity = self.allUnitVeloc[self.step + 1, i, :]

        # Update hawk states
        for i in range(self.numHawks):
            self.hawks[i].position = self.allHawkPositions[self.step + 1, i, :]
            self.hawks[i].velocity = self.allHawkVelocities[self.step + 1, i, :]

        self.step += 1


    def removeBoid(self, index):
        if 0 <= index < len(self.boids):
            # Remove boid from lists and update the total count
            del self.boids[index]
            self.allPositions = np.delete(self.allPositions, index, axis=1)
            self.allUnitVeloc = np.delete(self.allUnitVeloc, index, axis=1)
            self.numBoids -= 1


        


class Boid:

    def __init__(self, position: 'np.ndarray', velocity: 'np.ndarray', boidID, cohe, alin, sep, inertia = 0.5, speed = 1):
        self.position = position
        self.velocity = velocity
        self.boidID = boidID
        self.alignment = alin
        self.cohesion = cohe
        self.separation = sep
        self.sightRadius = 50
        self.speed = speed
        self.size = 1
        self.collisionTime = 0.3
        self.inertia = inertia

    def boidsInRadiusF(self, boidFlock: 'BoidFlock'):
        boidsInRadius = []
        for i in range(boidFlock.numBoids):
            if i != self.boidID:
                distance = np.linalg.norm(boidFlock.allPositions[boidFlock.step, i, :] - self.position)
                if distance < self.sightRadius:
                    boidsInRadius.append(i)
        return boidsInRadius
        
    def cohesionF(self, boidFlock: 'BoidFlock', boidsInRadius):
        # Return unit vector pointing from boid to average position of boids in the sight radius
        # Check if there are any boids in the radius
        if len(boidsInRadius) > 0:
            averagePosition = np.mean(boidFlock.allPositions[boidFlock.step, boidsInRadius, :], axis = 0)
            return (averagePosition - self.position) / np.linalg.norm(averagePosition - self.position)
        else:
            return np.array([0,0,0])

    def alignmentF(self, boidFlock: 'BoidFlock', boidsInRadius):
        # Return average heading of boids in the sight radius
        if len(boidsInRadius) == 0:
            return np.array([0,0,0])
        else:
            averageVelocity = np.mean(boidFlock.allUnitVeloc[boidFlock.step, boidsInRadius, :], axis = 0)

        averageVelocity = averageVelocity / np.linalg.norm(averageVelocity)

        alignVeloc = 0.95 * self.velocity + 0.05 * averageVelocity
        return alignVeloc / np.linalg.norm(alignVeloc)
    
    def separationF(self, boidFlock: 'BoidFlock', boidsInRadius):
        if len(boidsInRadius) > 0:
            averagePosition = np.mean(boidFlock.allPositions[boidFlock.step, boidsInRadius, :], axis = 0)
            vec2Average = averagePosition - self.position
            dist2Average = np.linalg.norm(vec2Average)
            vec2AvUnit = vec2Average / dist2Average

            sepProp = 5
            if dist2Average < self.size * sepProp:
                priority = 1/(dist2Average / sepProp)**2
            else:
                priority = 0

            planeNormal = np.cross(vec2AvUnit, self.velocity)
            avoidanceVector = np.cross(planeNormal, vec2AvUnit)
            avoidanceVector = priority * avoidanceVector / np.linalg.norm(avoidanceVector)
            return avoidanceVector
            
        else:
            return np.array([0,0,0])




    def movement(self, boidFlock: 'BoidFlock'):
        # Return the next position of the boid and unit vector of the next velocity

        # Find birds in the radius 
        boidsInRadius = self.boidsInRadiusF(boidFlock)

        # Run cohesion
        if self.cohesion != 0:
            cohesionVector = self.cohesionF(boidFlock, boidsInRadius)
        else:
            cohesionVector = np.array([0,0,0])

        if self.alignment != 0:
            alignmentVector = self.alignmentF(boidFlock, boidsInRadius)
        else:
            alignmentVector = np.array([0,0,0])

        if self.separation != 0:
            separationVector = self.separationF(boidFlock, boidsInRadius)
        else:
            separationVector = np.array([0,0,0])

        # Check if any of the entries in the vectors are NaN
            
        checkThese = np.concatenate((cohesionVector, alignmentVector, separationVector))
        for i in range(len(cohesionVector) * 3):
            if np.isnan(checkThese[i]):
                # Raise an error
                self.separationF(boidFlock, boidsInRadius)
                raise ValueError('One of the vectors is NaN')


        newVelocity = self.cohesion * cohesionVector + self.alignment * alignmentVector + self.separation * separationVector + self.inertia * self.velocity

        # Normalize the new velocity
        newVelocity = newVelocity / np.linalg.norm(newVelocity)

        newPosition = self.position + self.speed * newVelocity * boidFlock.dt

        if newPosition[0] < boidFlock.cubeC1[0]:
            newVelocity[0] = -newVelocity[0]
            newPosition[0] = boidFlock.cubeC1[0]
        elif newPosition[0] > boidFlock.cubeC2[0]:
            newVelocity[0] = -newVelocity[0]
            newPosition[0] = boidFlock.cubeC2[0]
        
        if newPosition[1] < boidFlock.cubeC1[1]:
            newVelocity[1] = -newVelocity[1]
            newPosition[1] = boidFlock.cubeC1[1]
        elif newPosition[1] > boidFlock.cubeC2[1]:
            newVelocity[1] = -newVelocity[1]
            newPosition[1] = boidFlock.cubeC2[1]
        
        if newPosition[2] < boidFlock.cubeC1[2]:
            newVelocity[2] = -newVelocity[2]
            newPosition[2] = boidFlock.cubeC1[2]
        elif newPosition[2] > boidFlock.cubeC2[2]:
            newVelocity[2] = -newVelocity[2]
            newPosition[2] = boidFlock.cubeC2[2]

        # if self.boidID == 0:
        #     print(self.position)
        #     print(newPosition)
        return newPosition, newVelocity
    
    
class HawkBoid:

     # Attirbutes:
    def __init__(self, position:'np.ndarray', velocity:'np.ndarray', hawkboidID) -> None:
        self.position = position
        self.velocity = velocity
        self.hawkboidID = hawkboidID
        self.huntingRadius = 50
        self.speed = 0.75
        self.size = 2

    # Methods
    def detectPrey(self, boidFlock: 'BoidFlock'):
        closest_boid_index = -1
        min_distance = self.huntingRadius  # Initialize with the maximum search radius

        for i, boid in enumerate(boidFlock.boids):
            distance = np.linalg.norm(boidFlock.allPositions[boidFlock.step, i, :] - self.position)
            if distance < min_distance:
                closest_boid_index = i
                min_distance = distance

        return closest_boid_index
    
    def movement(self, boidFlock: 'BoidFlock'):
        # Calculate the separation vector to avoid other hawks
        separation_vector = self.separation(boidFlock, min_separation_distance=10)
        # You may adjust min_separation_distance based on your simulation's scale
    
        # Find the prey
        preyIndex = self.detectPrey(boidFlock)
    
        # Define a vector that will influence the hawk's new direction
        direction_vector = np.array([0.0, 0.0, 0.0])
    
        if preyIndex == -1:
            # No prey found, move randomly but consider separation
            direction_vector = self.velocity + separation_vector
        else:
            # Prey found, calculate vector towards prey and consider separation
            preyPosition = boidFlock.allPositions[boidFlock.step, preyIndex, :]
            vec2Prey = preyPosition - self.position
            vec2PreyUnit = vec2Prey / np.linalg.norm(vec2Prey)
        
            direction_vector = vec2PreyUnit + separation_vector
    
        # Normalize the direction vector to ensure consistent speed
        if np.linalg.norm(direction_vector) > 0:
            direction_vector /= np.linalg.norm(direction_vector)
    
        # Calculate new position and velocity based on direction_vector
        newVelocity = direction_vector * self.speed
        newPosition = self.position + newVelocity * boidFlock.dt

        # Ensure the hawk doesn't phase into prey immediately by stopping at prey's position
        if preyIndex != -1 and np.linalg.norm(vec2Prey) < self.size:
            newPosition = self.position  # Hawk stops moving if it's close enough to catch prey
    
        return newPosition, newVelocity

        
    def killBoid(self, boidFlock: 'BoidFlock'):
        # Return the index of the prey boid
        for i in range(boidFlock.numBoids):
            distance = np.linalg.norm(boidFlock.allPositions[boidFlock.step, i, :] - self.position)
            if distance < self.size:
                return i
        return -1
    
    def separation(self, boidFlock: 'BoidFlock', min_separation_distance):

        move_away = np.array([0.0, 0.0, 0.0])
        nearby_hawks = 0

        for other_hawk in boidFlock.hawks:
            if other_hawk is not self:
                distance = np.linalg.norm(other_hawk.position - self.position)
                if distance < min_separation_distance:
                    # Calculate vector pointing away from the nearby hawk
                    move_away += self.position - other_hawk.position
                    nearby_hawks += 1

        if nearby_hawks > 0:
            move_away /= nearby_hawks  # Average the move away direction
            # Normalize the move away vector
            move_away /= np.linalg.norm(move_away)

        return move_away
        












#############          Old Separation Function         ###########################

    # def separationF(self, boidFlock: 'BoidFlock', boidsInRadius):
        
    #     futurePosSelf = self.position + self.collisionTime * self.velocity

    #     avoidanceList = np.full((len(boidsInRadius), 3), np.nan)
    #     npI = 0
    #     for i in boidsInRadius:
            
    #         # For each other boid, add the unit vector times the collision time to the position
    #         futurePosOther = boidFlock.allPositions[boidFlock.step, i, :] + self.collisionTime * boidFlock.allUnitVeloc[boidFlock.step, i, :]
    #         if np.linalg.norm(futurePosOther - futurePosSelf) < self.size:
    #             dist2Impact = self.speed * self.collisionTime
    #             vec2OtherImpact = futurePosOther - self.position
    #             vec2SelfImpact = futurePosSelf - self.position
    #             planeNormal = np.cross(vec2SelfImpact, vec2OtherImpact)
    #             avoidanceVector = np.cross(vec2SelfImpact, planeNormal) * 0.2 + vec2SelfImpact * 0.8
    #             avoidanceList[npI,:] = avoidanceVector / np.linalg.norm(avoidanceVector)
    #             npI += 1
    #         else:
    #             # Remove a row from the avoidance list
    #             avoidanceList = np.delete(avoidanceList, -1, 0)

    #     if npI > 0:
    #         for i in range(len(avoidanceList) - npI):
    #             for j in range(3):
    #                 if np.isnan(avoidanceList[i,j]):
    #                     print("bruu")

    #     if npI == 0:
    #         return np.array([0,0,0])
    #     else:
    #         return np.mean(avoidanceList, axis = 0) / np.linalg.norm(np.mean(avoidanceList, axis = 0))