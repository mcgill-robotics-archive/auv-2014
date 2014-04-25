#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')
import rospy

from std_msgs.msg import Float32
from blinky.msg import *
from blinky.srv import *

ledCount = 30

# Handles the ROS-specific communication with the service
# colors: array of RGB
def Battery_sendColors(colors):
    try:
        # wait for the blinky server node
        rospy.wait_for_service('blinky')

        # get access to the BatteryUpdateLights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('blinky', BatteryUpdateLights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "BatteryUpdateLights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

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
        color = RGB(255,255,255) #WHITE (shouldn't happen since voltage is < 25V)

    colors = [color]
    Battery_sendColors(colors)

def BatteryListener():
    rospy.init_node('battery_lights')
    rospy.Subscriber("electrical_interface/battery1_voltage", Float32, process_voltage)
    rospy.spin()

if __name__ == "__main__":
    BatteryListener()
