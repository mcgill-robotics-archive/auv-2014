import numpy as np
import math

def quaternionMatrix(q):
    #Any quaternion can be represented as a matrix and then we can
    #use matrix multiplication instead of quaternion multiplication
    return np.matrix([[q[0],-q[1],-q[2],-q[3]],
                      [q[1], q[0],-q[3], q[2]],
                      [q[2], q[3], q[0],-q[1]],
                      [q[3],-q[2], q[1], q[0]]])


def multiply(q,p):
    #Hamilton product
    return np.matrix([[float(p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3])]
                     ,[float(p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2])]
                     ,[float(p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1])]
                     ,[float(p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0])]])


def inverse(q):
    p = q.copy()
    p[1:4] *= -1
    return p


def quaternionFromRotationVector(omega):
    angle = math.sqrt(sum([x*x for x in omega]))
    if angle == 0:
        return np.matrix([[1],[0],[0],[0]])
    axis = [x/angle for x in omega]
    return np.matrix([[math.cos(angle/2)]] + [[x*math.sin(angle/2)] for x in axis])

def rotationVectorFromQuaternion(q):
    sin = math.sqrt(q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
    if sin == 0:
        return np.matrix([[0],[0],[0]])

    axis = q[1:4]/sin

    cos = q[0]

    angle = 2*math.atan2(sin,cos)

    return np.matrix(angle*axis)


def rotateVectorByQuaternion(vector, quaternion):
    p = np.matrix([[0],[vector[0]],[vector[1]],[vector[2]]])
    prod = multiply(quaternion, p)
    prod = multiply(prod,inverse(quaternion))
    return prod[1:4]


if __name__ == '__main__':
    x = [1, 0, 0]
    z = [0, 0, 1]
    q = quaternionFromRotationVector(z)
    rotated = rotateVectorByQuaternion(x, q)
    print(rotated)

