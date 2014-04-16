#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

from BlinkyAPI import *

# Recieves a voltage message and displays in on the blinky tape.
# Units are in volts, in floating point.
def process_voltage(voltage):
    volts = voltage.data
    color = RGB(0,0,0)

    if volts < 21:
        color = RGB(255,0,0)  #RED
    elif volts < 23.5:
        color = RGB(255,127,0) #YELLOW-ish
    elif volts < 25:
        color = RGB(0,255,0) #GREEN
    else:
        color = RGB(255,255,255) #RED (shouldn't happen since voltage is < 25V)

    colors = [color]
    Battery_sendColors(colors)

def BatteryListener():
    rospy.init_node('BatteryBlinky')
    rospy.Subscriber("electrical_interface/battery1_voltage", Float32, process_voltage)
    rospy.spin()

if __name__ == "__main__":
    BatteryListener()
