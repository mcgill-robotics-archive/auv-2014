import math
import numpy
import QuaternionUtils


def rotateThisByThat(toRotate, rotation):
    #Here we are assuming that these are numpy matrices
    angle = norm(rotation)
    if angle == 0:
        return toRotate.copy()
    axis = rotation/angle
    projection = axis*dot(toRotate, axis)
    remainder = toRotate - projection
    return projection + math.cos(angle)*remainder + math.sin(angle)*cross(axis, remainder)


def norm(vector):
    return math.sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2])


def dot(v1, v2):
    return v1[0:3].transpose()*v2[0:3]



def cross(v1, v2):
    return numpy.matrix([[int(v1[1]*v2[2] - v1[2]*v2[1])]
                        ,[int(v1[2]*v2[0] - v1[0]*v2[2])]
                        ,[int(v1[0]*v2[1] - v1[1]*v2[0])]])


def inverse(rotationVector):
    return -1*rotationVector


def composeRotations(r1, r2):
    #TODO: Work this over. We probably don't need quaternions
    q1 = QuaternionUtils.quaternionFromRotationVector(r1)
    q2 = QuaternionUtils.quaternionFromRotationVector(r2)
    q3 = QuaternionUtils.multiply(q1, q2)
    return QuaternionUtils.rotationVectorFromQuaternion(q3)