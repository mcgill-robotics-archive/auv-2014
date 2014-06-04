#!/usr/bin/env python

# TODO: DISTINGUISH PRACTICE FROM COMPETITION

# IMPORTS
from ctypes import *
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
NUMBER_OF_MICS = param.get_number_of_mics()
POS = param.get_mic_positions()
SPEED = param.get_speed()

# SET UP NODE AND TOPIC
rospy.init_node('solver')
solver_topic = rospy.Publisher('/hydrophones/sol',solution)
sol = solution()


def solve(data):
    """ Solves by multilateration """
    # PARSE TDOA
    dt = [0,data.tdoa_1,data.tdoa_2,data.tdoa_3]

    # MULTILATERATE
    A = np.zeros(NUMBER_OF_MICS)
    B = np.zeros(NUMBER_OF_MICS)
    C = np.zeros(NUMBER_OF_MICS)
    for i in range(2,NUMBER_OF_MICS):
        A[i] = (2*POS[i][0]) / (SPEED*dt[i]) - \
               (2*POS[1][0]) / (SPEED*dt[1])

        B[i] = (2*POS[i][1]) / (SPEED*dt[i]) - \
               (2*POS[1][1]) / (SPEED*dt[1])

        C[i] = SPEED*(dt[i] - dt[1]) - \
               (POS[i][0]**2 + POS[i][1]**2) / (SPEED*dt[i]) + \
               (POS[1][0]**2 + POS[1][1]**2) / (SPEED*dt[1])

    # SOLVE BY QR
    (x,y) = -np.linalg.solve(np.transpose([A[2:],B[2:]]),C[2:])

    # PUBLISH
    sol.cartesian.x = x
    sol.cartesian.y = y
    sol.polar.r = np.sqrt(x**2 + y**2)
    sol.polar.theta = np.degrees(np.arctan2(y,x))
    sol.target = True
    solver_topic.publish(sol)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/tdoa',tdoa,solve)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
