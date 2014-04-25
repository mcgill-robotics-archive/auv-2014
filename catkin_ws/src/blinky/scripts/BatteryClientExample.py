#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')
import rospy

from blinky.msg import *
from blinky.srv import *

from random import randint
import time

ledCount = 30

# Just an example. This returns an array of colors
# representing n in binary, with 1 being a random non-zero color
# and 0 being the RGB(0,0,0) color (led off)
def generate_colors(n):
    colors = []
    t = n

    for i in range(30):
	if n % 2 == 0:
	    colors.append(RGB(0,0,0))
	else:
	    colors.append(RGB(randint(0,255),randint(0,255),randint(0,255)))

	n = n / 2

    return colors

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

a = 0
while True:	
    Battery_sendColors(generate_colors(a))
    time.sleep(0.1)
    a = a + 1
