import numpy as np
from collections import namedtuple
ImpulseBody = namedtuple ('ImpulseBody', 'point mass inertia')

# My module
from collider import CollisionShape

gravAcceleration = [0, -9.81]
dragFactor = -0.05
angularDragFactor = 0.15
wall_e = 0.7
block_e = 0.9

def drag(velocity):
    return np.multiply(velocity, dragFactor)

def wallImpulse(velocity, normal, localPoint, mass, inertia):
    denominator = (
        1 / mass
        + (np.cross(localPoint, normal)**2) / inertia
    )
    return -(1 + wall_e) * np.dot(velocity, normal) / denominator

def blockImpulse(relativeVel, normal, oBody, cBody):
    denominator = (
        1 / oBody.mass
        + 1 / cBody.mass
        + (np.cross(oBody.point, normal)**2) / oBody.inertia
        + (np.cross(oBody.point, normal)**2) / oBody.inertia
    )
    return -(1 + block_e) * np.dot(relativeVel, normal) / denominator

def pointVelocity (rVel, localPoint):
    # rVel x localPoint, doing manually this part was super much faster than np.cross
    return (-localPoint[1] * rVel, localPoint[0] * rVel)

def blockCollisions(rigidbodies):
    bodyCount = len(rigidbodies)

    points = [rb.transformedPoints() for rb in rigidbodies]
    colliders = [CollisionShape(pts) for pts in points]

    # Current is B, has body/shape/edge/normal
    for current, collider in zip(rigidbodies, colliders):

        # Other is A, has points
        for other, worldPoints in zip (rigidbodies, points):

            for worldPoint in worldPoints:
                isInside, normal = collider.isPointInside(worldPoint)

                # There is collision
                if isInside:
                    # point local positions
                    cPoint = np.subtract(worldPoint, current.position)
                    oPoint = np.subtract(worldPoint, other.position)

                    # body velocities
                    cVel = current.velocity
                    oVel = other.velocity

                    # point velocities
                    cPointVel = np.add(cVel, pointVelocity(current.rVel, cPoint))
                    oPointVel = np.add(oVel, pointVelocity(other.rVel, oPoint))
                    relativeVel = np.subtract(oPointVel, cPointVel)

                    # Bodies are getting deeper in themselves
                    if np.dot(relativeVel, normal) < 0:
                        oBody = ImpulseBody(oPoint, other.mass, other.inertia)
                        cBody = ImpulseBody(cPoint, current.mass, current.inertia)
                        impulse = blockImpulse(relativeVel, normal, oBody, cBody)

                        current.addCollision(impulse, (-normal[0], -normal[1]), cPoint, other.id)
                        other.addCollision(impulse, normal, oPoint, current.id)

# end blockCollisions

def wallCollisions(rigidbodies):
    for rb in rigidbodies:

        for point in rb.transformedPoints():
            isOutside, normal = fieldCollider.isPointOutside(point)

            if isOutside:
                normal = (-normal[0], -normal[1])

                localPoint = np.subtract(point, rb.position)
                pointVel = np.add(rb.velocity, pointVelocity(rb.rVel, localPoint))

                # dot product of normal and velocity, to see if we also are still going deeper
                if np.dot(pointVel, normal) < 0:
                    impulse = wallImpulse(pointVel, normal, localPoint, rb.mass, rb.inertia)
                    rb.addCollision(impulse, normal, localPoint, -1)
# end wallCollisions
