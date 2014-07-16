#!/usr/bin/env python

# IMPORTS
import roslib
import rospy
import time
from math import ceil
from blinky.srv import *
from blinky.msg import *

# COLORS
BLACK = RGB(0, 0, 0)
CYAN = RGB(0, 255, 255)
GREEN = RGB(0, 255, 0)
ORANGE = RGB(255, 105, 0)
RED = RGB(255, 0, 0)
YELLOW = RGB(255, 200, 0)


def set_planner(colors):
    """ Lights up blinkytape for planner """
    try:
        rospy.wait_for_service('update_planner_lights')
        blinky_proxy = rospy.ServiceProxy('update_planner_lights', UpdatePlannerLights)
        result = blinky_proxy(colors)

        if result.success != 0:
            print 'UpdatePlannerLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def warning(state, frequency, color):
    """ Lights up blinkytape for warnings """
    try:
        rospy.wait_for_service('warning_lights')
        blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
        result = blinky_proxy([color], frequency, state)

        if result.success != 0:
            print 'WarningUpdateLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def countdown(time_counter, incomplete_color, complete_color, steps):
    """ Lights up blinkytape for countdown """
    start_time = time.time()
    time_left = time_counter
    while time_left > -0.1:
        colors = []
        incomplete_blinky_steps = int(ceil(steps * time_left / time_counter))

        for blinkies in range(incomplete_blinky_steps):
            colors.append(incomplete_color)
        for blinkies in range(steps - incomplete_blinky_steps):
            colors.append(complete_color)

        set_planner(colors)

        time_elapsed = time.time() - start_time
        time_left = time_counter - time_elapsed


if __name__ == '__main__':
    if not rospy.is_shutdown():
        while not rospy.has_param('/countdown/timeout'):
            pass

        timeout = int(rospy.get_param('/countdown/timeout'))

        # WARN BEFORE START
        time.sleep(timeout)
        set_planner([BLACK])
        warning(True, 1, YELLOW)
        time.sleep(5)
        set_planner([YELLOW])
        warning(False, 0, BLACK)

        # COUNTDOWN
        countdown(5, YELLOW, BLACK, 5)

        # WARN WHEN DONE
        warning(True, 2, GREEN)
        time.sleep(2.5)
        set_planner([GREEN])
        warning(False, 0, BLACK)
