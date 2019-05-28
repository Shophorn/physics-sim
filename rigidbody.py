import numpy as np

# My things
import maths
import physics
import geometry

class Rigidbody:
    count = 0

    def __init__(self, size, numPoints, density = 1.0, xPos = 0.0, yPos = 0.0, rPos = 0.0, xVel = 0.0, yVel = 0.0, rVel = 0.0):
        self.position = [xPos, yPos]
        self.rPos = rPos

        self.velocity = [xVel, yVel]
        self.rVel = rVel

        # Track id
        self.id = Rigidbody.count
        Rigidbody.count += 1

        self.collisions = []
        # self.size = size
        self.mass = geometry.area(numPoints, size)

        S = maths.scale(size)
        self.vertices = [np.matmul(S, point) for point in geometry.regularPolygon(numPoints)]
        self.inertia = self.mass * size**2

    def addCollision(self, impulse, normal, point, otherId):
        self.collisions.append((otherId, impulse, normal, point))

    def resolveCollisions(self):
        avgForces = {}

        # compute average from same objects, so we can better apprixmate multiple point hits
        for collision in self.collisions:
            otherId = collision[0]
            impulse = collision[1]
            normal = collision[2]
            point = collision[3]

            magnitude = impulse  / self.mass
            current = [
                normal[0] * magnitude,
                normal[1] * magnitude,
                (impulse / self.inertia) * np.cross(point, normal),
                1]   # count, which is used to divide to mean values

            if otherId in avgForces:
                avgForces[otherId] = np.add(avgForces[otherId], current)
            else:
                avgForces[otherId] = current

        for value in avgForces.values():
            self.velocity[0] += value[0] / value[3]
            self.velocity[1] += value[1] / value[3]
            self.rVel += value[2] / value[3]

        self.collisions = []

    def transformedPoints(self):
        T = maths.translation (self.position)
        R = maths.rotation(self.rPos)
        M = T @ R
        return [np.matmul(M, point)[0:2] for point in self.vertices]

    def update(self, deltaTime):
        # update velocities
        drag = physics.drag(self.velocity)
        self.velocity[0] += (drag[0] + physics.gravAcceleration[0]) * deltaTime
        self.velocity[1] += (drag[1] + physics.gravAcceleration[1]) * deltaTime
        self.rVel *= 1 - (physics.angularDragFactor * deltaTime)

        # update actual positions
        self.position[0] += self.velocity[0] * deltaTime
        self.position[1] += self.velocity[1] * deltaTime
        self.rPos += self.rVel * deltaTime
