#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def talker():
    pub = rospy.Publisher('chatter', Twist)
    rospy.init_node('talker')

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 1.0
    twist.linear.z = 1.0

    twist.angular.x = 1.0
    twist.angular.y = 1.0
    twist.angular.z = 1.0

    rospy.loginfo(twist)
    pub.publish(twist)
    rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
