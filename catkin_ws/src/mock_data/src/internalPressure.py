#!/usr/bin/env python

#__author__ = 'david lavoie-boutin'

import rospy
from std_msgs.msg import Float64
import random


def leak_publisher():
    pub = rospy.Publisher("internal_pressure", Float64)
    rospy.init_node('internal_pressure')
    leak = False

    while not rospy.is_shutdown():
        if not leak:
            pressure = 5
            if random.random()>0.97:
                leak = True
        else:
            pressure = 1

        rospy.loginfo(pressure)

        pub.publish(pressure)
        rospy.sleep(0.1)
    return str(pressure)
if __name__ == '__main__':
    try:
        leak_publisher()
    except rospy.ROSInterruptException:
        pass