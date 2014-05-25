### IMPORTS
import numpy as np
from objects import *

def solve(mics,speed):
    """ Solve by multilateration """
    NUMBER_OF_MICS = len(mics)

    A = np.zeros(NUMBER_OF_MICS)
    B = np.zeros(NUMBER_OF_MICS)
    C = np.zeros(NUMBER_OF_MICS)

    for i in range(2,NUMBER_OF_MICS):
        A[i] = (2*mics[i].pos.x) / (speed*mics[i].diff) - \
               (2*mics[1].pos.x) / (speed*mics[1].diff)

        B[i] = (2*mics[i].pos.y) / (speed*mics[i].diff) - \
               (2*mics[1].pos.y) / (speed*mics[1].diff)

        C[i] = speed*(mics[i].diff - mics[1].diff) - \
               (mics[i].pos.x**2 + mics[i].pos.y**2) / (speed*mics[i].diff) + \
               (mics[1].pos.x**2 + mics[1].pos.y**2) / (speed*mics[1].diff)

    (x,y) = -np.linalg.solve(np.transpose([A[2:],B[2:]]),C[2:])

    return Point(x,y)
