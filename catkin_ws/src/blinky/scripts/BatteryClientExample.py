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

    for i in range(15):
	if n % 2 == 0:
	    colors.append(RGB(0,0,0))
	else:
	    colors.append(RGB(randint(0,255),randint(0,255),randint(0,255)))

	n = n / 2

    return colors

# Handles the ROS-specific communication with the service
# colors: array of RGB
def Battery1_sendColors(colors):
    try:
        # wait for the update_battery1_lights service
        rospy.wait_for_service('update_battery1_lights')

        # get access to the UpdateBattery1Lights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('update_battery1_lights', UpdateBattery1Lights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "UpdateBattery1Lights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

def Battery2_sendColors(colors):
    try:
        # wait for the update_battery2_lights service
        rospy.wait_for_service('update_battery2_lights')

        # get access to the UpdateBattery2Lights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('update_battery2_lights', UpdateBattery2Lights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "UpdateBattery2Lights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

a = 0
while True:	
    Battery1_sendColors(generate_colors(a))
    Battery2_sendColors(generate_colors(a))
    time.sleep(0.1)
    a = a + 1
