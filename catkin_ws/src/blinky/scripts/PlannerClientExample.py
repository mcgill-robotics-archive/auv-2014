#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')
import rospy

from blinky.msg import *
from blinky.srv import *

ledCount = 30

# Handles the ROS-specific communication with the service
# colors: array of RGB
def Planner_sendColors(colors):
    try:
        # wait for the planner_update_lights service
        rospy.wait_for_service('planner_update_lights')

        # get access to the PlannerUpdateLights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('blinky', PlannerUpdateLights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "PlannerUpdateLights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

colors = [RGB(255,255,255), RGB(255,0,0), RGB(0,255,0)]
Planner_sendColors(colors)
