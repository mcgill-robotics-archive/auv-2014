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
RANGE = 2
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
LABELS = ['I', 'II', 'III', 'IV']

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
magn = [np.zeros(BUFFERSIZE/2 + 1) for channel in range(NUMBER_OF_MICS)]
peak = [0 for channel in range(NUMBER_OF_MICS)]


def update_magnitudes(data):
    """ Parses frequency amplitudes """
    magn[0] = data.channel_0
    magn[1] = data.channel_1
    magn[2] = data.channel_2
    magn[3] = data.channel_3


def update_peaks(data):
    """ Parses peak frequencies """
    peak[0] = data.channel_0
    peak[1] = data.channel_1
    peak[2] = data.channel_2
    peak[3] = data.channel_3


def analyze():
    """ Monitors target frequency and prints table """
    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    screen.addstr(0, 0, target)

    header = '\n %s\t%s\t%s\t\n\n' % ('MIC', 'FREQUENCY', 'MAGNITUDE')
    screen.addstr(header, curses.color_pair(3))

    for channel in range(NUMBER_OF_MICS):
        for i in range(TARGET_INDEX - RANGE, TARGET_INDEX + RANGE + 1):
            label = LABELS[channel]
            freq = i * FREQUENCY_PER_INDEX
            amplitude = magn[channel][i]
            string = ' %s\t%5d Hz\t%+4.2f dB \n' % (label, freq, amplitude)
            screen.addstr(string)

        screen.addstr('\n')


def maximize():
    """ Finds max frequency and prints table """
    screen.addstr(' MAX\t\t\t\t\t\n\n', curses.color_pair(3))
    for channel in range(NUMBER_OF_MICS):
        peak_index = int(peak[channel] / FREQUENCY_PER_INDEX)

        state = 0
        if np.abs(peak_index - TARGET_INDEX) <= RANGE:
            state = 2

        label = LABELS[channel]
        freq = peak[channel]
        amplitude = magn[channel][peak_index]
        string = ' %s\t%5d Hz\t%+4.2f dB \n' % (label, freq, amplitude)
        screen.addstr(string, curses.color_pair(state))


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
        rospy.Subscriber('/hydrophones/peak',peaks,update_peaks)

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
