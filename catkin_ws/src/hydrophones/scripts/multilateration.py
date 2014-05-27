#!/usr/bin/env python

# TODO: DISTINGUISH PRACTICE FROM COMPETITION 

# IMPORTS
from ctypes import *
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
from geometry_msgs.msg import Point
import param

# PARAMETERS
NUMBER_OF_MICS = param.get_number_of_mics()
SPEED = param.get_speed()
POS = param.get_mic_positions()
DEPTH_OF_PINGER = param.get_depth_of_pinger()


def solve(data):
    """ Solve by multilateration """
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
    sol = Point()
    sol.x = x
    sol.y = y
    sol.z = DEPTH_OF_PINGER
    solver_topic.publish(sol)
    

if __name__ == '__main__':
    try:
        rospy.init_node('solver')
        solver_topic = rospy.Publisher('/hydrophones/solution',Point)
        rospy.Subscriber('/hydrophones/time_difference',tdoa,solve)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
