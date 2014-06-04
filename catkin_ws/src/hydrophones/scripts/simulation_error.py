#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# SET UP NODE AND TOPIC
rospy.init_node('error')
error_topic = rospy.Publisher('/hydrophones/simulation/error',solution)
err = solution()


def compute_error(data):
    """ Computes and publishes error """
    (x,y) = param.get_simulation_solution()

    err.cartesian.x = x - data.cartesian.x
    err.cartesian.y = y - data.cartesian.y
    err.polar.r = np.sqrt(x**2 + y**2) - data.polar.r
    err.polar.theta = np.degrees(np.arctan2(y,x)) - data.polar.theta
    err.target = data.target
    error_topic.publish(err)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/solution',solution,compute_error)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass