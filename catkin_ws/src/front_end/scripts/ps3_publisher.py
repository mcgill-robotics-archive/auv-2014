#!/usr/bin/env python
#__author__ = 'david'

import rospy
from geometry_msgs.msg import Twist


def ps3_publisher(linear_x, linear_y, linear_z, pitch, yaw, roll):
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('ps3_publisher')

    twist = Twist()
    # define the twist message from the joystick input
    twist.linear.x = linear_x
    twist.linear.y = linear_y
    twist.linear.z = linear_z

    #TODO --> assign angular parameters to the corresponding axis
    twist.angular.x = roll
    twist.angular.y = pitch
    twist.angular.z = yaw

    rospy.loginfo(twist)

    pub.publish(twist)
    return str(twist)


if __name__ == '__main__':
    try:
        ps3_publisher(1, 2, 3, 4, 5, 6,)
    except rospy.ROSInterruptException:
        pass