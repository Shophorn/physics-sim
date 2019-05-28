import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import time
import math

# My own functions
import maths
import physics
from rigidbody import Rigidbody
from collider import CollisionShape
import geometry

# Field sizes
minX = 0
maxX = 100
minY = 0
maxY = 100

# Acting bodies in scene
rigidbodies = [
    Rigidbody(5.3, 4, xPos = maxX - 6, yPos = 15, xVel = 20, rVel = 10),
    Rigidbody(7.1, 5, xPos = maxX - 8, yPos = 50, yVel = 20, xVel = -100),
    Rigidbody(9.2, 4, xPos = 50, yPos = 9.2, rPos = 41, yVel = 100, xVel = 10),

    # These are so small that they easily move through edges in one time step
    # and don't collide properly. Otherwise they work well
    # Rigidbody(1.4, 3, xPos = 2, yPos = maxY -2),
    # Rigidbody(1.0, 3, xPos = 6, yPos = maxY -2),
    # Rigidbody(1.2, 5, xPos = 10, yPos = maxY -2),
    # Rigidbody(1.0, 3, xPos = 30, yPos = maxY -2),
    # Rigidbody(1.3, 4, xPos = 35, yPos = maxY -2),
    # Rigidbody(1.0, 4, xPos = 40, yPos = maxY -2),
    # Rigidbody(1.4, 5, xPos = 45, yPos = maxY -2),
    # Rigidbody(1.1, 4, xPos = 55, yPos = maxY -2),
    # Rigidbody(1.2, 5, xPos = 60, yPos = maxY -2),
    # Rigidbody(1.3, 4, xPos = 65, yPos = maxY -2),
    # Rigidbody(2.8, 5, xPos = 70, yPos = maxY -2),
    
    # Rigidbody(1.4, 3, xPos = 2, yPos = maxY -15),
    # Rigidbody(1.0, 3, xPos = 6, yPos = maxY -16),
    # Rigidbody(2.2, 4, xPos = 10, yPos = maxY -21),
    # Rigidbody(1.0, 3, xPos = 30, yPos = maxY -18),
    # Rigidbody(1.3, 4, xPos = 35, yPos = maxY -13),
    # Rigidbody(1.0, 4, xPos = 40, yPos = maxY -19),
    # Rigidbody(1.4, 4, xPos = 45, yPos = maxY -17),
    # Rigidbody(1.1, 4, xPos = 55, yPos = maxY -22),
    # Rigidbody(3.2, 5, xPos = 60, yPos = maxY -14),
    # Rigidbody(1.3, 4, xPos = 65, yPos = maxY -15),
    # Rigidbody(1.8, 5, xPos = 70, yPos = maxY -20),

    Rigidbody(2.1, 3, xPos = 95, yPos = maxY -2, xVel = -35),
    Rigidbody(3.4, 3, xPos = 4, yPos = 15, xVel = 50),
    Rigidbody(4.2, 4, xPos = 4, yPos = 23, xVel = 50),
    Rigidbody(3.1, 4, xPos = 4, yPos = 34, xVel = 50),
    Rigidbody(3.7, 3, xPos = 4, yPos = 55, xVel = 50),
    Rigidbody(5.7, 4, xPos = maxX - 5, yPos = 66, rPos = 2, yVel = 35, xVel = -50),
    Rigidbody(4.9, 4, xPos = 4, yPos = 78, xVel = 50),
    Rigidbody(9.2, 4, xPos = maxX / 2, yPos = maxY / 2, rVel = 3),

    # Metal boxes
    Rigidbody(3.5, 5, density = 7.0, xPos = 4, yPos = 46, xVel = 50, yVel = 10),
    Rigidbody(7.6, 4, density = 7.0, xPos = 4.2, yPos = 4.2, xVel = 50, yVel = 60, rVel = -5),
    Rigidbody(4.3, 4, density = 7.0, xPos = 20, yPos = maxY - 3, xVel = 20),
    Rigidbody(5.0, 3, density = 7.0, xPos = 50, yPos = maxY - 2, xVel = -50)
]
objectCount = len(rigidbodies)
metalBoxCount = 4

# Make two items rectangle
rectPoints = [[-0.8, -0.6, 1], [-0.8, 0.6, 1], [0.8, 0.6, 1], [0.8, -0.6, 1]]
S = maths.scale(4.9)
rectPoints = [np.matmul(S, point) for point in rectPoints]
rigidbodies[-6].vertices = rectPoints

rectPoints = [[-0.97, -0.24, 1], [-0.97, 0.24, 1], [0.97, 0.24, 1], [0.97, -0.24, 1]]
S = maths.scale(9.2)
rectPoints = [np.matmul(S, point) for point in rectPoints]
rigidbodies[-5].vertices = rectPoints

# Make one item weird
S = maths.scale(5.7)
weirdPoints = [np.matmul(S, point) for point in [[0.595, 0.902, 1], [0.939, -0.848, 1], [-0.633, -0.326, 1], [-0.903, 0.27, 1]]]
rigidbodies[-7].vertices = weirdPoints

# This keeps stuff inside
physics.fieldCollider = CollisionShape([(minX, minY), (minX, maxY), (maxX, maxY), (maxX, minY)])

# This gives similar shades in different hues
onValue = 0.95
offValue = 0.37
colors = [
    (onValue, offValue, offValue),   # 0 : red
    (onValue, onValue, offValue),    # 1 : yellow
    (offValue, onValue, offValue),   # 2 : green
    (offValue, onValue, onValue),    # 3 : cyan
    (offValue, offValue, onValue),   # 4 : blue
    (onValue, offValue, onValue)     # 5 : magenta
]
metalColor = (0.4, 0.4, 0.4)

# SIMULATION FUNCTIONS
fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(8, 8)

ax = plt.axes()
patches = [None] * objectCount

def init():
    for i in range(objectCount):
        patches[i] = plt.Polygon(rigidbodies[i].transformedPoints())

        if i < objectCount - metalBoxCount:
            patches[i].set_facecolor(colors[i % len(colors)])
        else:
            patches[i].set_facecolor(metalColor)

        ax.add_patch(patches[i])
    return patches


# Time between frames
lastTime = 0.0

pause = False
doPauseImmediately = False

# SIMULATION LOOP
def simulate(_):
    global doPauseImmediately
    global pause

    global lastTime
    currentTime = time.process_time()
    deltaTime = currentTime - lastTime
    lastTime = currentTime

    # Use slower time to make simulation more accurate
    # deltaTime /= 3

    # Simulate
    if not pause:
        physics.blockCollisions(rigidbodies)
        physics.wallCollisions(rigidbodies)

        for i in range(objectCount):
            rigidbodies[i].resolveCollisions()
            rigidbodies[i].update(deltaTime)

    # Draw
    for i in range(objectCount):
        patches[i].xy = rigidbodies[i].transformedPoints()

    if doPauseImmediately:
        doPauseImmediately = False
        pause = True

    return patches

# EVENT: click to pause
def onClick(event):
    global pause
    pause ^= True
fig.canvas.mpl_connect('button_press_event', onClick)

# EVENT: press 'w' to step single frame
def onPress(event):
    if event.key == 'w':
        global pause
        global doPauseImmediately

        pause = False
        doPauseImmediately = True
fig.canvas.mpl_connect('key_press_event', onPress)

# Do Animation
anim = animation.FuncAnimation(fig, simulate, init_func=init, interval=1, blit=True)

plt.xticks(np.arange(minX, maxX + 1, step=20))
plt.yticks(np.arange(minY, maxY + 1, step=20))
plt.show()
