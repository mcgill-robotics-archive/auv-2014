#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
param.set_simulation_parameters()
NUMBER_OF_MICS = param.get_number_of_mics()
POS = param.get_mic_positions()
SPEED = param.get_speed()

# SET UP NODE AND TOPIC
rospy.init_node('tdoa_sim')
tdoa_topic = rospy.Publisher('/hydrophones/sim/tdoa',tdoa)
rate = rospy.Rate(0.5)
dt = tdoa()


def compute_tdoa():
    """ Computes and publishes expected TDOA """
    (x,y) = param.get_simulation_target()

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
