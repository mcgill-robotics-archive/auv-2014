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

# SET UP NODE AND TOPIC
rospy.init_node('solver')
cartesian_topic = rospy.Publisher('/hydrophones/solution/cartesian',Point)
cylindrical_topic = rospy.Publisher('/hydrophones/solution/cylindrical',cylindrical)
cartesian = Point()
cylindrical = cylindrical()


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

    # PUBLISH CARTESIAN COORDINATES
    cartesian.x = x
    cartesian.y = y
    cartesian.z = DEPTH_OF_PINGER
    cartesian_topic.publish(cartesian)

    # PUBLISH CYLINDRICAL COORDINATES
    cylindrical.r = np.sqrt(x**2 + y**2)
    cylindrical.theta = np.degrees(np.arctan2(y,x))
    cylindrical.z = DEPTH_OF_PINGER
    cylindrical_topic.publish(cylindrical)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/time_difference',tdoa,solve)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
