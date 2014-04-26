#!/usr/bin/env python
import rospy

# For the Lock object
import threading

# To measure elapsed time during warning displays
import time

# Get access to the blinky services and messages
from std_msgs.msg import Float32
from blinky.msg import *
from blinky.srv import *

import BlinkyTape as bt

# number of leds per blinky segment
ledCount = 30

# blinky tape class handle (accesses 2 segments)
# change first argument with usb serial port
# where the tape is found
blt = bt.BlinkyTape("/dev/blinkyTape", 2 * ledCount)

# The two blinky tape segments
planner_colorList = []
battery_colorList = []

# Warnings on the planner segment
warning_colors = []

# Frequency of the warning display in Hz
warning_freq = 0.0

# Flags indicating whether to display a warning
warning_on = False

# set all leds to (0,0,0)
def initialize_blinkies():
    global planner_colorList	# need to declare as global to preserve
    global battery_colorList	# across function calls
    colors = []
	
    for i in range(ledCount):
    	colors.append(RGB(139,69,19)) # Brown (!!!)

    # store the initial states
    planner_colorList = colors
    battery_colorList = colors

    # set all leds off
    for i in range(2 * ledCount):
    	blt.sendPixel(0,0,0)

    blt.show()

# callback methods during service request from client
# req is a request containing the service arguments
# as fields accessible with the '.'

# planner_colorList and battery_colorList store
# the state of the blinky tape colors.
# The planner segment is closest to the Arduino
# The battery segment is farthest from the Arduino
# |Arduino|--------- Planner ---------||--------- Battery ---------|

# Update Planner segment
# req.colors: list of RGB colors to display
def update_planner(req):
    global planner_colorList
    lock = threading.Lock()

    with lock:
        planner_colorList = req.colors

    return UpdatePlannerLightsResponse(0)

# Update Battery segment
# req.colors: list of RGB colors to display
def update_battery(req):
    global battery_colorList
    lock = threading.Lock()

    with lock:
        battery_colorList = req.colors

    return UpdateBatteryLightsResponse(0)

# Display a warning on the planner segment
# req.colors: list of colors to display
# req.frequency: frequency at which to flash the warning (in Hz)
# req.on: activate warning or stop it
def warn_lights(req):
    global warning_colors
    global warning_freq
    global warning_on

    with lock:
        warning_on = req.on
        warning_colors = req.colors
        warning_freq = req.frequency

    return WarningLightsResponse(0)

def BlinkyTapeServer():
    initialize_blinkies()
    rospy.init_node('blinky')
    upl = rospy.Service('update_planner_lights', UpdatePlannerLights, update_planner)
    ubl = rospy.Service('update_battery_lights', UpdateBatteryLights, update_battery)
    wl = rospy.Service('warning_lights', WarningLights, warn_lights)

    lock = threading.Lock()
    edge_time = time.time()
    state = 0   # alternates between 0 (planner display) and 1 (warning display)
    list1 = planner_colorList   # List of colors to display on segment 1
    list2 = battery_colorList   # List of colors to display on segment 2

    # Print the current state
    while not rospy.is_shutdown():
        
        # get stable copies of the (volatile) lists
        # if warnings are on, alternate between planner colors
        # and warning colors, after measuring the time period.
        if (warning_on == False) or (warning_freq <= 0.0):
            with lock:
                list1 = planner_colorList

            # reset time counter
            edge_time = time.time()
            
        # if warnings are on, toggle the planner display after each half-period
        elif (warning_freq > 0.0) and (time.time() - edge_time >= 0.5 / warning_freq):
            if state == 0:
                with lock:
                    list1 = warning_colors

                state = 1
            else:
                with lock:
                    list1 = planner_colors

                state = 0
            
            # reset time counter
            edge_time = time.time()

        with lock:
            list2 = battery_colorList

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
