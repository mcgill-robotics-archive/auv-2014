#!/usr/bin/env python

import rospy

# For the Lock object
import threading

# Get access to the blinky services and messages
from blinky.msg import *
from blinky.srv import *
import BlinkyTape as bt

# number of leds per blinky segment
ledCount = 30

# blinky tape class handle (accesses 2 segments)
# change first argument with usb serial port
# where the tape is found
blt = bt.BlinkyTape("/dev/ttyACM0", 2 * ledCount)

# The two blinky tape segments
blinky1_colorList = []
blinky2_colorList = []

# return success/error codes
SUCCESS = 0
UNKNOWN_ID = 1
WRONG_COLORS_COUNT = 2

# set all leds to (0,0,0)
def initialize_blinkies():
	global blinky1_colorList	# need to declare as global to preserve
	global blinky2_colorList	# across function calls
	colors = []
	
	for i in range(ledCount):
		colors.append(RGB(139,69,19))

	# store the initial states
	blinky1_colorList = colors
	blinky2_colorList = colors

	# set all leds off
	for i in range(2 * ledCount):
		blt.sendPixel(0,0,0)

	blt.show()

# callback method during service request from client
# req is a request containing the service arguments
# as fields accessible with the '.'

# blinky1_colorList and blinky2_colorList store
# the state of the blinky tape colors

# blinkyID specifies which blinky to access.
#	1 means the closest and 2 the furthest
#	(in an Arduino-centered coordinate system)

def update_colors(req):
	global blinky1_colorList
	global blinky2_colorList
	colorList = req.btColors	# get the service parameters from
	blinkyID = req.blinkyID		# the request

	lock = threading.Lock()
	
	# Check if id is valid
	if blinkyID == 1:
		with lock:
			blinky1_colorList = colorList
	elif blinkyID == 2:
		with lock:
			blinky2_colorList = colorList
	else:
		return BlinkyTapeServiceResponse(UNKNOWN_ID)

	return BlinkyTapeServiceResponse(SUCCESS)

def BlinkyTapeServer():
	initialize_blinkies()
	rospy.init_node('BlinkyDisplay')
	s = rospy.Service('BlinkyDisplay', BlinkyTapeService, update_colors)

	lock = threading.Lock()
	
	# Print the current state
	while not rospy.is_shutdown():
		# get stable copy of the (volatile) lists
		with lock:
			list1 = blinky1_colorList
			list2 = blinky2_colorList

		length1 = len(list1)
		length2 = len(list2)

		# send the rgb colors in the display buffer
		# the n-th call to sendPixel before show() sets the n-th led.
		# repeat the same pattern until it fills up all leds
		for m in range(ledCount):
			if length1 == 0:
				break
			
			rgb = list1[m % length1]
			blt.sendPixel(rgb.r, rgb.g, rgb.b)
			
		for n in range(ledCount):
			if length2 == 0:
				break
			
			rgb = list2[n % length2]
			blt.sendPixel(rgb.r, rgb.g, rgb.b)

		# actually print the led colors on the tape
		blt.show()

if __name__ == "__main__":
	BlinkyTapeServer()
