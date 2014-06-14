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
param.set_parameters()
NUMBER_OF_MICS = param.get_number_of_mics()
BUFFERSIZE = param.get_buffersize()
SAMPLING_FREQUENCY = param.get_sampling_frequency()
TARGET_FREQUENCY = param.get_target_frequency()

# USEFUL CONSTANTS
FREQUENCY_RANGE = 100       # RANGE OFF TARGET TO MONITOR   Hz
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TIME_PER_INDEX = 1 / SAMPLING_FREQUENCY
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
RANGE = int(round(FREQUENCY_RANGE / FREQUENCY_PER_INDEX))
RFFT_LENGTH = BUFFERSIZE / 2 + 1
LABELS = ['I', 'II', 'III', 'IV']

# SET UP NODE AND TOPIC
rospy.init_node('table')

# SET UP CURSES
curses.initscr()
curses.echo()
height = NUMBER_OF_MICS * (2 * RANGE + 4) + 8
screen = curses.newwin(height, 100)
screen.clear()
curses.start_color()
curses.use_default_colors()
# WARNING
curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_RED)
# SUCCESS
curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)
# HEADING
curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)


class mic(object):
    ''' Microphone class '''
    __slots__ = ['label', 'time', 'freq', 'magn']

    def __init__(self, quadrant):
        ''' Initializes mic object '''
        self.reset(quadrant)

    def reset(self, quadrant):
        ''' Resets mic object '''
        self.label = LABELS[quadrant]
        self.time = np.zeros(BUFFERSIZE, np.float)
        self.freq = np.zeros(RFFT_LENGTH, np.float)
        self.magn = np.zeros(RFFT_LENGTH, np.float)

    def compute_fft(self):
        ''' FFTs time domain '''
        self.freq = np.fft.rfft(self.time)

    def compute_magnitude(self):
        ''' Computes magnitude '''
        np.seterr(divide='ignore')
        self.magn = 20*np.log10(np.abs(self.freq))


# CREATE MIC OBJECTS
mics = [mic(i) for i in range(NUMBER_OF_MICS)]


def read(data):
    ''' Reads signal from microphones '''
    mics[0].time = data.channel_0
    mics[1].time = data.channel_1
    mics[2].time = data.channel_2
    mics[3].time = data.channel_3


def process():
    ''' FFTs time domain and computes magnitude '''
    for i in range(NUMBER_OF_MICS):
        mics[i].compute_fft()
        mics[i].compute_magnitude()


def analyze():
    ''' Monitors target frequency and prints table '''
    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    screen.addstr(0, 0, target)

    header = '\n%s\t%s\t%s\t\n\n' % (' MIC', 'FREQUENCY', 'MAGNITUDE')
    screen.addstr(header, curses.color_pair(3))

    for i in range(NUMBER_OF_MICS):
        for j in range(TARGET_INDEX - RANGE,
                       TARGET_INDEX + RANGE + 1):
            value = '%s\t%5d Hz\t%+4.2f\tdB\n' % \
                    (' ' + mics[i].label,
                     j * FREQUENCY_PER_INDEX,
                     mics[i].magn[j])
            screen.addstr(value)

        screen.addstr('\n')


def maximize():
    ''' Finds max frequency and prints table '''
    screen.addstr(' MAX\t\t\t\t\t\n\n', curses.color_pair(3))
    for i in range(NUMBER_OF_MICS):
        max = np.argmax(mics[i].magn)

        state = 0
        if ((max >= TARGET_INDEX - RANGE) and
            (max <= TARGET_INDEX + RANGE)):
            state = 2

        value = '%s\t%5d Hz\t%+4.2f\tdB\n' % \
                (' ' + mics[i].label,
                 max * FREQUENCY_PER_INDEX,
                 mics[i].magn[max])
        screen.addstr(value, curses.color_pair(state))


def update(data):
    """ Updates vizualization """
    read(data)
    process()
    analyze()
    maximize()
    screen.refresh()


if __name__ == '__main__':
    # MAIN
    try:
        rospy.Subscriber('/hydrophones/audio',channels,update)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


    # QUIT PEACEFULLY
    try:
        screen.addstr('\n GOODBYE \n\n', curses.color_pair(1))
        screen.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
