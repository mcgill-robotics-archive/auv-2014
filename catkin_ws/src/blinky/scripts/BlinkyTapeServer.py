#!/usr/bin/env python

import rospy
from blinky.msg import *
from blinky.srv import *
import BlinkyTape as bt

ledCount = 30
blt = bt.BlinkyTape("/dev/ttyACM0", 2 * ledCount)

#The two blinky tape segments
blinky1_colorList = []
blinky2_colorList = []

SUCCESS = 0
UNKNOWN_ID = 1
WRONG_COLORS_COUNT = 2

def initialize_blinkies():
	global blinky1_colorList
	global blinky2_colorList
	colors = []
	
	for i in range(ledCount):
		colors.append(RGB(0,0,0))

	blinky1_colorList = colors
	blinky2_colorList = colors

def handle_BlinkyTape(req):
	global blinky1_colorList
	global blinky2_colorList
	colorList = req.btColors
	blinkyID = req.blinkyID

	if blinkyID == 1:
		blinky1_colorList = colorList.data
	elif blinkyID == 2:
		blinky2_colorList = colorList.data
	else:
		return BlinkyTapeServiceResponse(UNKNOWN_ID)

	if len(colorList.data) != ledCount:
		return BlinkyTapeServiceResponse(WRONG_COLORS_COUNT)

	for rgb in blinky1_colorList:
		blt.sendPixel(rgb.r, rgb.g, rgb.b)
	
	for rgb in blinky2_colorList:
		blt.sendPixel(rgb.r, rgb.g, rgb.b)

	blt.show()
	return BlinkyTapeServiceResponse(SUCCESS)

def BlinkyTapeServer():
	initialize_blinkies()
	rospy.init_node('BlinkyTapeServer')
	s = rospy.Service('BlinkyTapeServer', BlinkyTapeService, handle_BlinkyTape)
	rospy.spin()

if __name__ == "__main__":
	BlinkyTapeServer()
