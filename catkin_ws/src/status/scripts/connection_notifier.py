#!/usr/bin/env python

# IMPORTS
import roslib
import rospy
import os
import time
from blinky.srv import *
from blinky.msg import *
rospy.init_node('connection')

# COLORS
BLACK = RGB(0, 0, 0)
CYAN = RGB(0, 255, 255)
GREEN = RGB(0, 255, 0)
ORANGE = RGB(255, 105, 0)
PURPLE = RGB(255, 0, 105)
RED = RGB(255, 0, 0)
WHITE = RGB(255, 255, 255)
YELLOW = RGB(255, 200, 0)

# TIMER
timing = False
start_time = time.time()
end_time = 0
delta_time = 0

def warning(state, color):
    """ Lights up BlinkyTape for warnings """
    try:
        rospy.wait_for_service('warning_lights')
        blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
        result = blinky_proxy([color], 2, state)

        if result.success != 0:
            print 'WarningUpdateLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def get_connection_status():
    """ Checks if planner is running and computer is untethered """
    connection = open('/sys/class/net/eth0/operstate').read().rstrip()

    return connection


if __name__ == '__main__':
    if not rospy.is_shutdown():
        try:
            start_time = time.time()

            # WAIT
            while get_connection_status() != "up":
                pass

            end_time = time.time()
            delta_time = end_time - start_time

            rospy.loginfo("Time until connection established: %f seconds", delta_time)

            os.system("bash -ic blinky_alert")

        except:
            pass
