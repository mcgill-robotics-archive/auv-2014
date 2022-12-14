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
NUMBER_OF_PINGERS = 2
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
LABELS = ['I', 'II', 'III', 'IV']

# SET UP NODE
rospy.init_node('table')

# SET UP CURSES
curses.initscr()
curses.echo()
curses.cbreak()

# SET UP HEADER
top = curses.newwin(6, 100, 2, 5)
top.clear()

# SET UP LEFT COLUMN
height = NUMBER_OF_MICS * (2 * RANGE + 3) + 1
left_column = curses.newwin(height, 50, 8, 5)
left_column.clear()

# SET UP RIGHT COLUMN
right_column = curses.newwin(height, 50, 8, 55)
right_column.clear()

# SET UP FOOTER
bottom = curses.newwin(30, 100, height + 6, 5)
bottom.clear()

# SET UP CURSES COLORS
curses.start_color()
curses.use_default_colors()
curses.init_pair(1,curses.COLOR_BLACK,curses.COLOR_RED)     # WARNING
curses.init_pair(2,curses.COLOR_WHITE,curses.COLOR_BLUE)    # SUCCESS
curses.init_pair(3,curses.COLOR_BLACK,curses.COLOR_WHITE)   # HEADING

# STORE DATA
magn = [np.zeros(BUFFERSIZE/2 + 1) for channel in range(NUMBER_OF_MICS)]
peak = [0 for channel in range(NUMBER_OF_MICS)]
dt = [0 for channel in range(NUMBER_OF_MICS-1)]
sim_dt = [0 for channel in range(NUMBER_OF_MICS-1)]
sol = [{'x': 0, 'y': 0, 'r': 0, 'theta': 0, 'new': False} for pinger in range(NUMBER_OF_PINGERS)]
sim_sol = [{'x': 0, 'y': 0, 'r': 0, 'theta': 0, 'new': False} for pinger in range(NUMBER_OF_PINGERS)]
last_ping = [time.time() for pinger in range(NUMBER_OF_PINGERS)]


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


def update_sim_tdoas(data):
    """ Parses simulation TDOAs """
    sim_dt[0] = data.tdoa_1
    sim_dt[1] = data.tdoa_2
    sim_dt[2] = data.tdoa_3


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


def update_parameters():
    """ Updates parameters on the fly """
    global SIMULATION, SNR, TARGET_FREQUENCY, TARGET_INDEX
    SIMULATION = param.get_simulation_state()
    TARGET_FREQUENCY = param.get_target_frequency()
    TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))

    if SIMULATION:
        sim_sol[0]['x'],sim_sol[0]['y'] = param.get_simulation_target()
        sim_sol[1]['x'],sim_sol[1]['y'] = param.get_simulation_dummy()
        SNR = param.get_snr()

    for pinger in range(NUMBER_OF_PINGERS):
        x,y = sim_sol[pinger]['x'],sim_sol[pinger]['y']
        sim_sol[pinger]['r'] = np.sqrt(x**2 + y**2)
        sim_sol[pinger]['theta'] = np.degrees(np.arctan2(y,x))


def header_table():
    """ Draws header table """
    header = 'HYDROPHONES MONITOR'
    top.addstr(0, 0, header.center(90), curses.color_pair(3))
    target = 'MONITORING %4d Hz' % (TARGET_FREQUENCY)
    top.addstr(2, 0, target.center(90))
    sim_state = 'SIMULATION %s' % ('ON' if SIMULATION else 'OFF')
    top.addstr(3, 0, sim_state.center(90))
    if SIMULATION:
        snr = 'SNR %4.1f dB' % (SNR)
        top.addstr(4, 0, snr.center(90))

    top.refresh()


def frequency_table():
    """ Draws table of relevant frequencies """
    header = ' %s\t%s\t%s\t\n\n' % ('MIC', 'FREQUENCY', 'MAGNITUDE')
    left_column.addstr(0, 0, header, curses.color_pair(3))

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
    header = ' PEAK FREQUENCY\t\t\t\t\n\n'
    right_column.addstr(0, 0, header, curses.color_pair(3))

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
    header = '\n TIME DIFFERENCE OF ARRIVAL\t\t\n\n'
    right_column.addstr(header, curses.color_pair(3))

    for channel in range(NUMBER_OF_MICS-1):
        state = 0
        if SIMULATION and np.abs(dt[channel] - sim_dt[channel]) <= 5e-6:
            state = 2

        string = ' %s\t\t\t%+1.9f\t\n' % (LABELS[channel], dt[channel])
        right_column.addstr(string, curses.color_pair(state))

    right_column.refresh()


def solution_table():
    """ Draws table of solutions """
    x, y, r, theta = [], [], [], []
    for pinger in range(NUMBER_OF_PINGERS):
        x.append(' X\t\t\t%+4.2f\t\t\n' % (sol[pinger]['x']))
        y.append(' Y\t\t\t%+4.2f\t\t\n' % (sol[pinger]['y']))
        r.append(' R\t\t\t%+4.2f\t\t\n' % (sol[pinger]['r']))
        theta.append(' THETA\t\t\t%+4.2f\t\t\n' % (sol[pinger]['theta']))

    header = ['\n TARGET PINGER\t\t', '\n DUMMY PINGER\t\t']

    for pinger in range(NUMBER_OF_PINGERS):
        state = [3, 0, 0, 0, 0]
        if sol[pinger]['new']:
            last_ping[pinger] = time.time()
            sol[pinger]['new'] = False

        timer = '%1.1f' % (time.time() - last_ping[pinger])
        header[pinger] += '   %s s \n\n' % (timer.rjust(10))

        if SIMULATION:
            if np.abs(sol[pinger]['x'] - sim_sol[pinger]['x']) <= 2:
                state[1] = 2
            if np.abs(sol[pinger]['y'] - sim_sol[pinger]['y']) <= 2:
                state[2] = 2
            if np.abs(sol[pinger]['r'] - sim_sol[pinger]['r']) <= 5:
                state[3] = 2
            if np.abs(sol[pinger]['theta'] - sim_sol[pinger]['theta']) <= 3:
                state[4] = 2

        right_column.addstr(header[pinger], curses.color_pair(state[0]))
        right_column.addstr(x[pinger], curses.color_pair(state[1]))
        right_column.addstr(y[pinger], curses.color_pair(state[2]))
        right_column.addstr(r[pinger], curses.color_pair(state[3]))
        right_column.addstr(theta[pinger], curses.color_pair(state[4]))

    right_column.refresh()


def plot():
    """ Plots frequencies of the first hydrophone """
    x_length, y_length = 90, 10
    for x in range(x_length):
        for y in range(y_length):
            if magn[0][TARGET_INDEX-x_length/2+x] >= y_length*(y_length-y):
                bottom.addch(y,x,ord(' '),curses.color_pair(3))
            else:
                bottom.addch(y,x,ord(' '))

    bottom.refresh()


def update_visualization():
    """ Updates visualization """
    update_parameters()
    header_table()
    frequency_table()
    peak_table()
    tdoa_table()
    solution_table()
    plot()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/magn',channels,update_magnitudes)
        rospy.Subscriber('/hydrophones/peak',peaks,update_peaks)
        rospy.Subscriber('/hydrophones/tdoa',tdoa,update_tdoas)
        rospy.Subscriber('/hydrophones/sim/tdoa',tdoa,update_sim_tdoas)
        rospy.Subscriber('/hydrophones/sol',solution,update_solution)

        while not rospy.is_shutdown():
            update_visualization()

    except Exception as e:
        bottom.addstr('\n\n')
        bottom.addstr(str(e).center(90), curses.color_pair(1))
        bottom.addstr('\n')

    try:
        bottom.addstr('\n\n')
        bottom.addstr('GOODBYE'.center(90), curses.color_pair(1))
        bottom.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
