#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')

import rospy

from blinky.msg import *
from blinky.srv import *

ledCount = 30

PLANNER_ID = 1
BATTERY_ID = 2

# Handles the ROS-specific communication with the service
# colors:	array of RGB	
# blinkyID:	blinky segment the colors should be sent to
def send_colorList(colors, blinkyID):
	try:
		rospy.wait_for_service('BlinkyDisplay')
		BlinkyDisplay = rospy.ServiceProxy('BlinkyDisplay', BlinkyTapeService)
		res = BlinkyDisplay(colors, blinkyID)

		if res.success != 0:
			print "BlinkyDisplay request unsuccessful: %s"%res

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def Planner_sendColors(colors):
	send_colorList(colors,1)

def Battery_sendColors(colors):
	send_colorList(colors,2)
