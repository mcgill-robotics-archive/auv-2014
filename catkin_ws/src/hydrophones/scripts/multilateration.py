#!/usr/bin/env python

# IMPORTS
from ctypes import *
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
from std_msgs.msg import Bool, Float64
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
yaw_topic = rospy.Publisher('/hydrophones/target',Float64)
sol = solution()
yes = Bool()
yaw = Float64()


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

    # QUICK FIX FOR OVERFLOW
    x_axis_delay = dt[1]
    y_axis_delay = dt[3]
    WIDTH, HEIGHT = param.get_mic_dimensions()
    corrected = False
    if x_axis_delay > 0 and y_axis_delay > 0:
        if x > WIDTH/2 and y > HEIGHT/2:
            x = -np.abs(x)
            y = -np.abs(y)
            corrected = True
            rospy.logfatal('CORRECTED FOR QUADRANT III')
    elif x_axis_delay < 0 and y_axis_delay < 0:
        if x < WIDTH/2 and y < HEIGHT/2:
            x = np.abs(x)
            y = np.abs(y)
            corrected = True
            rospy.logfatal('CORRECTED FOR QUADRANT I')
    elif x_axis_delay < 0 and y_axis_delay > 0:
        if x < WIDTH/2 and y > HEIGHT/2:
            x = np.abs(x)
            y = -np.abs(y)
            corrected = True
            rospy.logfatal('CORRECTED FOR QUADRANT IV')
    elif x_axis_delay > 0 and y_axis_delay < 0:
        if x > WIDTH/2 and y < HEIGHT/2:
            x = -np.abs(x)
            y = np.abs(y)
            corrected = True
            rospy.logfatal('CORRECTED FOR QUADRANT II')

    # PUBLISH SOLUTION
    sol.cartesian.x = x
    sol.cartesian.y = y
    sol.polar.r = np.sqrt(x**2 + y**2)
    sol.polar.theta = np.degrees(np.arctan2(y,x))
    sol.target = data.target
    solver_topic.publish(sol)

    # PUBLISH IF THE PINGER BELOW US
    if not corrected and sol.target and sol.polar.r <= 1.0:
        yes.data = True
    else:
        yes.data = False
    are_we_there_yet_topic.publish(yes)

    # PUBLISH YAW WITH FIX FOR COORDINATE SYSTEM
    if sol.target:
        theta = 90 - sol.polar.theta
        if theta > 180:
            theta -= 360
        yaw.data = np.radians(theta)
        yaw_topic.publish(yaw)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/tdoa',tdoa,solve)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
