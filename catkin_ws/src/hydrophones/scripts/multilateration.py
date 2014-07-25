#!/usr/bin/env python

# TODO: DISTINGUISH PRACTICE FROM COMPETITION

# IMPORTS
from ctypes import *
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
from std_msgs.msg import Bool
import param

# PARAMETERS
try:
    NUMBER_OF_MICS = param.get_number_of_mics()
    POS = param.get_mic_positions()
    SPEED = param.get_speed()
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('solver')
solver_topic = rospy.Publisher('/hydrophones/sol',solution)
are_we_there_yet_topic = rospy.Publisher('/hydrophones/are_we_there_yet',Bool)
sol = solution()
yes = Bool()


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

    # PUBLISH SOLUTION
    sol.cartesian.x = x
    sol.cartesian.y = y
    sol.polar.r = np.sqrt(x**2 + y**2)
    sol.polar.theta = np.degrees(np.arctan2(y,x))
    sol.target = data.target
    solver_topic.publish(sol)

    # PUBLISH IF THE PINGER BELOW US
    if sol.target and sol.polar.r <= 0.5:
        yes.data = True
    else:
        yes.data = False
    are_we_there_yet_topic.publish(yes)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/tdoa',tdoa,solve)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
