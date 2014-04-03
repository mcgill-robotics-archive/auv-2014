#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from BlinkyAPI import *

def process_voltage(data):
    rospy.loginfo("I recieved %s", data.data)
    
def BatteryListener():
    rospy.init_node('BatteryBlinky')
    rospy.Subscriber("arduino/battery_voltage1", Float32, process_voltage)
    rospy.spin()

if __name__ == 'main':
    BatteryListener()
