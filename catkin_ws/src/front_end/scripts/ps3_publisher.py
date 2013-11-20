#!/usr/bin/env python
#__author__ = 'david'

import rospy
from geometry_msgs.msg import Twist


def ps3_publisher(lx, ly, lz, pitch, yaw, roll, frequency):
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('ps3_publisher')

    twist = Twist()
    # define the twist message from the joystick input
    twist.linear.x = lx
    twist.linear.y = ly
    twist.linear.z = lz

    #TODO --> assign angular parameters to the corresponding axis
    twist.angular.x = roll
    twist.angular.y = pitch
    twist.angular.z = yaw

    rospy.loginfo(twist)

    pub.publish(twist)
    return str(twist)


if __name__ == '__main__':
    try:
        ps3_publisher(1, 2, 4, 5, 6, 1)
    except rospy.ROSInterruptException:
        pass