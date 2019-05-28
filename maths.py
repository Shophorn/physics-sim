import math
import numpy as np

def rotation(angle):
    sin = math.sin (angle)
    cos = math.cos (angle)

    return np.array([
        [cos, -sin, 0],
        [sin, cos, 0],
        [0, 0, 1]
    ])

def translation(vec):
    return np.array([
        [1, 0, vec[0]],
        [0, 1, vec[1]],
        [0, 0, 1]
    ])

def scale(size):
    return np.array([
        [size, 0, 0],
        [0, size, 0],
        [0, 0, 1]
    ])
