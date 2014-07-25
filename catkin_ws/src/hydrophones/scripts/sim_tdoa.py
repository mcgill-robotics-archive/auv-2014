#!/usr/bin/env python

# IMPORTS
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
rospy.init_node('tdoa_sim')
tdoa_topic = rospy.Publisher('/hydrophones/sim/tdoa',tdoa)
dt = tdoa()


def compute_tdoa(target):
    """ Computes and publishes expected TDOA """
    if target.data:
        (x,y) = param.get_simulation_target()
    else:
        (x,y) = param.get_simulation_dummy()

    times = []
    for i in range(NUMBER_OF_MICS):
        magnitude = np.sqrt((POS[i][0]-x)**2 + (POS[i][1]-y)**2)
        times.append(magnitude/SPEED)

    dt.tdoa_1 = times[1] - times[0]
    dt.tdoa_2 = times[2] - times[0]
    dt.tdoa_3 = times[3] - times[0]
    dt.target = target.data
    tdoa_topic.publish(dt)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/sim/target_active',Bool,compute_tdoa)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
