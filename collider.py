import numpy as np

class CollisionShape:
    def __init__ (self, points):
        # Points must be going clockwise order, and shape must be convex, not going to check though
        self.points = points
        self.numPoints = len(points)

        # Initialize normals so we can use them against all checks
        self.normals = [None] * self.numPoints
        for i in range (self.numPoints):
            p0 = self.points[i]
            p1 = self.points[(i + 1) % self.numPoints]

            edge = np.subtract(p1, p0)
            magnitude = np.sqrt(edge[0]**2 + edge[1]**2)
            normal = (-edge[1] / magnitude, edge[0] / magnitude)

            self.normals [i] = normal

    def isPointInside(self, point):
        # actually the one closest to zero
        biggestDot = -10000000000000
        biggestIndex = -1

        isInside = True

        for i in range(self.numPoints):
            r = np.subtract(point, self.points[i])
            dot = np.dot(r, self.normals[i])

            if dot > 0:
                return False, None

            if -0.0001 > dot > biggestDot:
                biggestDot = dot
                biggestIndex = i

        return True, self.normals[biggestIndex]

    def isPointOutside(self, point):
        smallestDot = 10000000000000
        smallestIndex = -1

        isOutside = False

        for i in range(self.numPoints):
            r = np.subtract(point, self.points[i])
            dot = np.dot(r, self.normals[i])

            if dot > 0:
                isOutside = True

            if 0.0001 < dot < smallestDot:
                smallestDot = dot
                smallestIndex = i

        return isOutside, self.normals[smallestIndex]
