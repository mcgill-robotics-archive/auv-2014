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

# Color to separate subsections in the planner segment
separation_color = RGB(0,0,0)

# The two blinky tape segments
planner_colorList = []
battery_colorList = []

# Warning displayed on both segments
warning_colorList = []

# Frequency of the warning display in Hz
warning_freq = 0.0

# Flags indicating whether to display a warning
warning_on = False

# set all leds to (0,0,0)
def initialize_blinkies():
    global planner_colorList	# need to declare as global to preserve
    global battery_colorList	# across function calls
    colors = []

    # store the initial states
    for i in range(ledCount):
    	planner_colorList.append(RGB(0,255,255)) # Cyan (!!!)
        battery_colorList.append(RGB(0,255,255))

    # set all leds off
    for i in range(2 * ledCount):
    	blt.sendPixel(0,0,0)

    blt.show()

# callback methods during service request from client
# req is a request containing the service arguments
# as fields accessible with the '.'

# Update Planner segment
# req.colors: list of RGB colors to display

# This method divides the planner segment into n subsections,
# where n is the number of colors passed in the req array.
# Each color indicates the color of one of these subsections,
# and the subsections are separated by a black led.
# The planner segment consists of two parallel
# strips of 15 leds each (totaling 30). The second strip is
# the reverse of the first.
def update_planner(req):
    global planner_colorList
    global separation_color
    lock = threading.Lock()

    segments = len(req.colors)
    seg_size = 16/segments # 16 comes from magic (works for 1 <= i <= 5)

    colors = []

    for s in req.colors:
        for i in range(seg_size - 1):
            colors.append(s)
        colors.append(separation_color)

    del(colors[-1])

    # filler with the last color until all 15 leds are on
    for j in range(len(colors),15):
        colors.append(req.colors[-1])

    # reverse the colors on the second half of 15 leds
    for j in range(15):
        colors.append(colors[14-j])

    with lock:
        planner_colorList = colors

    return UpdatePlannerLightsResponse(0)

# Update Battery1 segment
# req.colors: list of RGB colors to display
def update_battery1(req):
    global battery_colorList
    lock = threading.Lock()

    if len(req.colors) != 15:
        return UpdateBattery1LightsResponse(1)

    with lock:
        for i in range(15):
            battery_colorList[i] = req.colors[i]

    print planner_colorList
    return UpdateBattery1LightsResponse(0)

# Update Battery2 segment
# The colors are reversed.
def update_battery2(req):
    global battery_colorList
    lock = threading.Lock()

    if len(req.colors) != 15:
        return UpdateBattery2LightsResponse(1)

    with lock:
        for i in range(15):
            battery_colorList[29-i] = req.colors[i]

    print planner_colorList
    return UpdateBattery2LightsResponse(0)

# Display a warning on the planner and battery segments
# req.colors: list of colors to display
# req.frequency: frequency at which to flash the warning (in Hz)
# req.on: activate warning or stop it
def warn_lights(req):
    global warning_colorList
    global warning_freq
    global warning_on
    lock = threading.Lock()

    with lock:
        warning_on = req.on
        warning_colorList = req.colors
        warning_freq = req.frequency

    return WarningLightsResponse(0)

def BlinkyTapeServer():
    initialize_blinkies()
    rospy.init_node('blinky')
    upl = rospy.Service('update_planner_lights', UpdatePlannerLights, update_planner)
    ub1l = rospy.Service('update_battery1_lights', UpdateBattery1Lights, update_battery1)
    ub2l = rospy.Service('update_battery2_lights', UpdateBattery2Lights, update_battery2)
    wl = rospy.Service('warning_lights', WarningLights, warn_lights)
    pub_planner = rospy.Publisher('planner_colors', RGBArray)
    pub_battery = rospy.Publisher('battery_colors', RGBArray)
    pub_warning = rospy.Publisher('warning_colors', RGBArray)

    lock = threading.Lock()
    edge_time = time.time()
    state = 0   # alternates between 0 (normal display) and 1 (warning display)
    list1 = planner_colorList   # List of colors to display on segment 1
    list2 = battery_colorList   # List of colors to display on segment 2

    # Print the current state
    while not rospy.is_shutdown():
        # get stable copies of the (volatile) lists
        # get current warning state. Keep a copy to avoid the risk
        # of warning_freq changing to 0 while testing the if condition
        # thereby causing a division by zero.
        with lock:
            planner_colorList_copy = planner_colorList
            battery_colorList_copy = battery_colorList
            warning_on_copy = warning_on
            warning_freq_copy = warning_freq
            warning_colorList_copy = warning_colorList

        # publish all color lists
        pub_planner.publish(planner_colorList_copy)
        pub_battery.publish(battery_colorList_copy)
        pub_warning.publish(warning_colorList_copy)

        # if warnings are on, alternate between planner/battery colors
        # and warning colors, after measuring the time period.
        planner_length = len(planner_colorList_copy)
        battery_length = len(battery_colorList_copy)
        if (warning_on_copy == False) or (warning_freq_copy <= 0.0):
            list1 = planner_colorList_copy[0:planner_length/2] + battery_colorList_copy[0:battery_length/2]
            list2 = battery_colorList_copy[battery_length/2:battery_length] + planner_colorList_copy[planner_length/2:planner_length]
            edge_time = time.time() # reset time counter

        # if warnings are on, toggle the planner and battery display after each half-period
        elif (warning_freq_copy > 0.0) and (time.time() - edge_time >= 0.5 / warning_freq_copy):
            if state == 0:
                list1 = warning_colorList_copy
                list2 = warning_colorList_copy
                state = 1
            else:
                list1 = planner_colorList_copy[0:planner_length/2] + battery_colorList_copy[0:battery_length/2]
                list2 = battery_colorList_copy[battery_length/2:battery_length] + planner_colorList_copy[planner_length/2:planner_length]
                state = 0

            # reset time counter
            edge_time = time.time()

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
