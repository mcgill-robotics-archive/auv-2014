#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from geometry_msgs.msg import Twist


def ps3_publisher(linear_x, linear_y, linear_z, pitch, yaw, roll, topic):
    pub = rospy.Publisher(topic, Twist)
    #rospy.init_node('ps3_publisher')

    twist = Twist()
    # define the twist message from the joystick input
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    #TODO --> assign angular parameters to the corresponding axis
    twist.angular.x = roll
    twist.angular.y = pitch
    twist.angular.z = yaw

    rospy.loginfo(twist)

    pub.publish(twist)
    return str(twist)
