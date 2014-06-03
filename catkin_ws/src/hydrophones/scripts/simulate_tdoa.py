#!/usr/bin/env python

# TODO: ADD NOISE
#       USE SNR CORRECTLY
#       3 DIMENSIONS
#       SIMULATE BOTH PRACTICE AND COMPETITION

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
param.set_simulation_parameters()
NUMBER_OF_MICS = param.get_number_of_mics()
SPEED = param.get_speed()
POS = param.get_mic_positions()
DEPTH_OF_PINGER = param.get_depth_of_pinger()

# SET UP NODE AND TOPIC
rospy.init_node('tdoa')
tdoa_topic = rospy.Publisher('/hydrophones/simulation/tdoa',tdoa)
rate = rospy.Rate(0.5)
dt = tdoa()


def compute_tdoa():
    """ Computes and publishes expected TDOA """
    (x,y) = param.get_simulation_solution()

    times = []
    for i in range(NUMBER_OF_MICS):
        magnitude = np.sqrt((POS[i][0]-x)**2 + (POS[i][1]-y)**2)
        times.append(magnitude/SPEED)

    dt.tdoa_1 = times[1] - times[0]
    dt.tdoa_2 = times[2] - times[0]
    dt.tdoa_3 = times[3] - times[0]
    dt.target = True
    tdoa_topic.publish(dt)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            compute_tdoa()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass