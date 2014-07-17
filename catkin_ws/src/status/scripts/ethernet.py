#!/usr/bin/env python

# IMPORTS
import roslib
import rospy
import os
import time
from blinky.srv import *
from blinky.msg import *
rospy.init_node('connection')
rate = rospy.Rate(10)

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
            while not rospy.has_param('/connection/'):
                pass
            debug = rospy.get_param('/connection/debug')
            if debug:
                rospy.logwarn('DEBUG MODE ACTIVE')

            on_boot = True
            connection_is_good = False

            start_time = time.time()
            while not rospy.is_shutdown():
                if connection_is_good and get_connection_status() == "down":
                    start_time = time.time()
                    rospy.logwarn("Connection dropped!")
                    connection_is_good = False
                    if debug:
                        warning(True, RED)
                elif not connection_is_good and get_connection_status() == "up":
                    end_time = time.time()
                    delta_time = end_time - start_time
                    rospy.logwarn("Connection established: %f s", delta_time)
                    connection_is_good = True
                    if on_boot:
                        on_boot = False
                        os.system("bash -ic blinky_alert")
                    elif debug:
                        warning(True, GREEN)
                rate.sleep()

        except:
            pass

    if debug:
        os.system("bash -ic stop_warning")
