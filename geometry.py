from numpy import matmul
from math import pi
from math import cos, sin
import maths

def regularPolygon(numPoints):
    # Make sure to rotate clockwise
    R = maths.rotation(-1 * 2 * pi / numPoints)
    vertices = []

    # Use homogeneous coordinates
    vertex = [0, 1, 1]
    for i in range(numPoints):
        vertices.append(vertex)
        vertex = matmul(R, vertex).tolist()
    return vertices

def area(numPoints, size):
    a = pi / numPoints
    return numPoints * cos(a) * sin(a) * size**2
