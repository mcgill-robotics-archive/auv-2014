#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# SET UP NODE AND TOPIC
try:
    rospy.init_node('error')
    error_topic = rospy.Publisher('/hydrophones/sim/error',solution)
    err = solution()
except:
    print 'ROS NOT RUNNING'
    exit(1)


def compute_error(data):
    """ Computes and publishes error """
    if data.target:
        (x,y) = param.get_simulation_target()
    else:
        (x,y) = param.get_simulation_dummy()

    err.cartesian.x = x - data.cartesian.x
    err.cartesian.y = y - data.cartesian.y
    err.polar.r = np.sqrt(x**2 + y**2) - data.polar.r
    err.polar.theta = np.degrees(np.arctan2(y,x)) - data.polar.theta
    err.target = data.target
    error_topic.publish(err)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/sol',solution,compute_error)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass