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
except:
    print 'ROS NOT RUNNING'
    exit(1)

# USEFUL CONSTANTS
RANGE = 2
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
LABELS = ['I', 'II', 'III', 'IV']

# SET UP NODE
rospy.init_node('table')

# SET UP CURSES
curses.initscr()
curses.echo()
curses.cbreak()

height = NUMBER_OF_MICS * (2 * RANGE + 4)
left_column = curses.newwin(height, 50, 2, 5)
left_column.clear()

right_column = curses.newwin(height, 50, 2, 55)
right_column.clear()

bottom = curses.newwin(3, 100, height, 5)
bottom.clear()

curses.start_color()
curses.use_default_colors()
curses.init_pair(1,curses.COLOR_BLACK,curses.COLOR_RED)     # WARNING
curses.init_pair(2,curses.COLOR_WHITE,curses.COLOR_BLUE)    # SUCCESS
curses.init_pair(3,curses.COLOR_BLACK,curses.COLOR_WHITE)   # HEADING

# STORE DATA
magn = [np.zeros(BUFFERSIZE/2 + 1) for channel in range(NUMBER_OF_MICS)]
peak = [0 for channel in range(NUMBER_OF_MICS)]
dt = [0 for channel in range(NUMBER_OF_MICS-1)]
sol = [{'x': 0, 'y': 0, 'r': 0, 'theta': 0, 'new': False} for pinger in range(2)]


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


def update_tdoas(data):
    """ Parses TDOAs """
    dt[0] = data.tdoa_1
    dt[1] = data.tdoa_2
    dt[2] = data.tdoa_3


def update_solution(data):
    """ Parses solution """
    pinger = 1
    if data.target:
        pinger = 0

    sol[pinger]['x'] = data.cartesian.x
    sol[pinger]['y'] = data.cartesian.y
    sol[pinger]['r'] = data.polar.r
    sol[pinger]['theta'] = data.polar.theta
    sol[pinger]['new'] = True
    sol[1-pinger]['new'] = False


def frequency_table():
    """ Draws table of relevant frequencies """
    global TARGET_FREQUENCY, TARGET_INDEX
    TARGET_FREQUENCY = param.get_target_frequency()
    TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))

    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    left_column.addstr(0, 0, target)

    header = '\n %s\t%s\t%s\t\n\n' % ('MIC', 'FREQUENCY', 'MAGNITUDE')
    left_column.addstr(header, curses.color_pair(3))

    for channel in range(NUMBER_OF_MICS):
        for i in range(TARGET_INDEX - RANGE, TARGET_INDEX + RANGE + 1):
            label = LABELS[channel]
            freq = i * FREQUENCY_PER_INDEX
            amplitude = magn[channel][i]
            string = ' %s\t%5d Hz\t%+4.2f dB \n' % (label, freq, amplitude)
            left_column.addstr(string)

        left_column.addstr('\n')

    left_column.refresh()


def peak_table():
    """ Draws table of peak frequencies """
    header = ' MAX FREQUENCIES\t\t\t\n\n'
    right_column.addstr(2, 0, header, curses.color_pair(3))

    for channel in range(NUMBER_OF_MICS):
        peak_index = int(peak[channel] / FREQUENCY_PER_INDEX)

        state = 0
        if np.abs(peak_index - TARGET_INDEX) <= RANGE:
            state = 2

        label = LABELS[channel]
        freq = peak[channel]
        amplitude = magn[channel][peak_index]
        string = ' %s\t%5d Hz\t%+4.2f dB\t\n' % (label, freq, amplitude)
        right_column.addstr(string, curses.color_pair(state))

    right_column.refresh()


def tdoa_table():
    """ Draws table of solutions """
    header = '\n TDOA\t\t\t\t\t\n\n'
    right_column.addstr(header, curses.color_pair(3))

    for channel in range(NUMBER_OF_MICS-1):
        string = ' %s\t\t%+1.9f\t\t\n' % (LABELS[channel], dt[channel])
        right_column.addstr(string)

    right_column.refresh()


def solution_table():
    """ Draws table of solutions """
    global SIMULATION
    SIMULATION = param.get_simulation_state()

    x, y, r, theta = [], [], [], []
    for pinger in range(2):
        x.append(' X\t\t%4.2f\t\t\t\n' % (sol[pinger]['x']))
        y.append(' Y\t\t%4.2f\t\t\t\n' % (sol[pinger]['y']))
        r.append(' R\t\t%4.2f\t\t\t\n' % (sol[pinger]['r']))
        theta.append(' Theta\t\t%4.2f\t\t\t\n' % (sol[pinger]['theta']))

    header = ['\n TARGET SOLUTION\t\t\t\n\n', '\n DUMMY SOLUTION\t\t\t\t\n\n']

    for pinger in range(2):
        state = 0
        if sol[pinger]['new']:
            state = 2

        right_column.addstr(header[pinger], curses.color_pair(3))
        right_column.addstr(x[pinger], curses.color_pair(state))
        right_column.addstr(y[pinger], curses.color_pair(state))
        right_column.addstr(r[pinger], curses.color_pair(state))
        right_column.addstr(theta[pinger], curses.color_pair(state))

    right_column.refresh()


def update_visualization():
    """ Updates visualization """
    frequency_table()
    peak_table()
    tdoa_table()
    solution_table()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/magn',channels,update_magnitudes)
        rospy.Subscriber('/hydrophones/peak',peaks,update_peaks)
        rospy.Subscriber('/hydrophones/tdoa',tdoa,update_tdoas)
        rospy.Subscriber('/hydrophones/sol',solution,update_solution)

        while not rospy.is_shutdown():
            update_visualization()

    except rospy.ROSInterruptException:
        pass

    try:
        bottom.addstr(' GOODBYE\t\t\t\t\t\t\t\t\t\t  \n\n', curses.color_pair(1))
        bottom.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
