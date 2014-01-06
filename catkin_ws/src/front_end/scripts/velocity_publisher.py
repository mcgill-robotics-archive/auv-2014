#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def velocity_publisher(linear_x, linear_y, z_position, pitch, yaw, vel_topic, z_pos_des_topic):
    vel_pub = rospy.Publisher(vel_topic, Twist)
    zdes_pub = rospy.Publisher(z_pos_des_topic, Float64)
    #rospy.init_node('velocity_publisher')

    twist = Twist()
    # define the twist message from the joystick input
    twist.linear.x = linear_x
    twist.linear.y = linear_y

    #TODO --> assign angular parameters to the corresponding axis
    twist.angular.y = pitch
    twist.angular.z = yaw

    rospy.loginfo(twist)

    vel_pub.publish(twist)
    zdes_pub.publish(z_position)

    return str(twist)
