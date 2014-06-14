#!/usr/bin/env python

# IMPORTS
from ctypes import *
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import curses
import os
import time
import sys
import param

# PARAMETERS
try:
    NUMBER_OF_MICS = param.get_number_of_mics()
    BUFFERSIZE = param.get_buffersize()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    TARGET_FREQUENCY = param.get_target_frequency()

except:
    print 'ROS NOT RUNNING'
    exit(1)

# USEFUL CONSTANTS
FREQUENCY_RANGE = 100       # RANGE OFF TARGET TO MONITOR   Hz
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TIME_PER_INDEX = 1 / SAMPLING_FREQUENCY
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
RANGE = int(round(FREQUENCY_RANGE / FREQUENCY_PER_INDEX))

# SET UP NODE
rospy.init_node('table')

# SET UP CURSES
curses.initscr()
curses.echo()
curses.cbreak()
height = NUMBER_OF_MICS * (2 * RANGE + 4) + 8
screen = curses.newwin(height, 100, 2, 5)
screen.clear()
curses.start_color()
curses.use_default_colors()
curses.init_pair(1,curses.COLOR_BLACK,curses.COLOR_RED)     # WARNING
curses.init_pair(2,curses.COLOR_WHITE,curses.COLOR_BLUE)    # SUCCESS
curses.init_pair(3,curses.COLOR_BLACK,curses.COLOR_WHITE)   # HEADING

# STORE DATA
magn = [np.zeros(BUFFERSIZE/2 + 1) for i in range(NUMBER_OF_MICS)]


def update_magnitudes(data):
    """ Parses frequency amplitudes """
    magn[0] = data.channel_0
    magn[1] = data.channel_1
    magn[2] = data.channel_2
    magn[3] = data.channel_3


def analyze():
    """ Monitors target frequency and prints table """
    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    screen.addstr(0, 0, target)

    header = '\n%s\t%s\t%s\t\n\n' % (' MIC', 'FREQUENCY', 'MAGNITUDE')
    screen.addstr(header, curses.color_pair(3))

    for i in range(NUMBER_OF_MICS):
        for j in range(TARGET_INDEX - RANGE,
                       TARGET_INDEX + RANGE + 1):
            value = ' %d\t%5d Hz\t%+4.2f\tdB\n' % \
                    (i, j * FREQUENCY_PER_INDEX, magn[i][j])
            screen.addstr(value)

        screen.addstr('\n')


def maximize():
    """ Finds max frequency and prints table """
    screen.addstr(' MAX\t\t\t\t\t\n\n', curses.color_pair(3))
    for i in range(NUMBER_OF_MICS):
        max = np.argmax(magn[i])

        state = 0
        if ((max >= TARGET_INDEX - RANGE) and
            (max <= TARGET_INDEX + RANGE)):
            state = 2

        value = ' %d\t%5d Hz\t%+4.2f\tdB\n' % \
                (i, max * FREQUENCY_PER_INDEX, magn[i][max])
        screen.addstr(value, curses.color_pair(state))


def update_visualization():
    """ Updates visualization """
    global TARGET_FREQUENCY, TARGET_INDEX
    TARGET_FREQUENCY = param.get_target_frequency()
    TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))

    analyze()
    maximize()
    screen.refresh()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/magn',channels,update_magnitudes)

        while not rospy.is_shutdown():
            update_visualization()

    except rospy.ROSInterruptException:
        pass

    try:
        screen.addstr('\n GOODBYE \n\n', curses.color_pair(1))
        screen.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
