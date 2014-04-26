#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')
import rospy

import time

from std_msgs.msg import *
from blinky.msg import *
from blinky.srv import *

ledCount = 30

# Handles the ROS-specific communication with the service
# colors: array of RGB
# frequency: frequency of warning display in Hz
# on: activate warning or stop it
def warn(colors, frequency, on):
    try:
        # wait for the warning_lights service
        rospy.wait_for_service('warning_lights')

        # get access to the WarningLights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('blinky', WarningLights)

        # call service
        res = blinky_proxy(colors, frequency, on)

        if res.success != 0:
            print "WarningUpdateLights request unsuccessful: %s"%res

    except Exception as e:
        print "\n%s: %s" % ('Exception', e)
 
# Callback function which checks the topic and sends a warning
# for 10 seconds if the value is larger than 100.0
def process_topic(value):
    colors = [RGB(255,0,0)]
    freq = 10.0

    if value > 100.0:
       warn(colors, freq, True)
       time.sleep(10)
       warn([RGB(0,0,0)], 0, False)

def warning_example_listener():
    rospy.init_node('warning_example')
    rospy.Subscriber("warning_example_topic", Float32, process_topic)
    rospy.spin()

if __name__ == "__main__":
    warning_example_listener()
