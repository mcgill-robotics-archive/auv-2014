#!/usr/bin/env python

#__author__ = 'david lavoie-boutin'

import rospy
from std_msgs.msg import Float64
import random


def voltage_publisher():
    pub = rospy.Publisher("battery_voltage", Float64)
    rospy.init_node('battery_voltage')
    leak = False

    while not rospy.is_shutdown():
        if not leak:
            voltage = 5
            if random.random()>0.97:
                leak = True
        else:
            voltage = 1

        rospy.loginfo(voltage)

        pub.publish(voltage)
        rospy.sleep(0.1)
    return str(voltage)
if __name__ == '__main__':
    try:
        voltage_publisher()
    except rospy.ROSInterruptException:
        pass