#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')

import rospy

import time

from blinky.msg import *
from blinky.srv import *

def generate_colors(n):
	colors = []
	
	for i in range(30):
		if i == n:
			colors.append(RGB(255,0,0))
		else:
			colors.append(RGB(0,0,0))
	
	colorList = RGBArray(colors)
	return colorList

def BlinkyTape_Client():
	while True:
		for i in range(30):
			startTime = time.time()
			while time.time() - startTime < 0.007:
				try:
					rospy.wait_for_service('BlinkyTapeServer')
					BlinkyTapeServer = rospy.ServiceProxy('BlinkyTapeServer', BlinkyTapeService)
					
					res = BlinkyTapeServer(generate_colors(i),1)
					#res.success += BlinkyTapeServer(generate_colors(i),2).success

					if res.success != 0:
						print "BlinkyTapeServer request unsuccessful: %s"%res

				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
			

if __name__ == "__main__":
	print "Sending colors to blinky tape!!!"
	BlinkyTape_Client()
