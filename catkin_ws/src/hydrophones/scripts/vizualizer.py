#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param
import curses
import time

# PARAMETERS
NUMBER_OF_MICS = param.get_number_of_mics()
BUFFERSIZE = param.get_buffersize()
SAMPLING_FREQUENCY = param.get_sampling_frequency()
TARGET_FREQUENCY = param.get_target_frequency()
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
DIVISION = 5

# SET UP NODE
rospy.init_node('vizualizer')
rate = rospy.Rate(SAMPLING_FREQUENCY/BUFFERSIZE)
peak = [0 for channel in range(NUMBER_OF_MICS)]
data = [[] for channel in range(NUMBER_OF_MICS)]
magn = [[] for channel in range(NUMBER_OF_MICS)]

# SET UP CURSES
curses.initscr()
curses.echo()
curses.cbreak()
HEIGHT = 40
WIDTH = 115
screen = curses.newwin(HEIGHT, WIDTH, 0, 3)
screen.clear()
curses.start_color()
curses.use_default_colors()
curses.init_pair(1,curses.COLOR_BLACK,curses.COLOR_RED)     # WARNING
curses.init_pair(2,curses.COLOR_WHITE,curses.COLOR_BLUE)    # SUCCESS
curses.init_pair(3,curses.COLOR_BLACK,curses.COLOR_WHITE)   # HEADING


def update_magn(magnitudes):
    """ Updates magnitudes """
    data[0] = magnitudes.channel_0
    data[1] = magnitudes.channel_1
    data[2] = magnitudes.channel_2
    data[3] = magnitudes.channel_3

    for channel in range(NUMBER_OF_MICS):
        magn[channel] = []
        sum = 0
        for frequency in range(len(data[channel])):
            sum += data[channel][frequency]
            if frequency % DIVISION == 0:
                magn[channel].append(sum / DIVISION)
                sum = 0


def plot():
    """ Updates vizualization """
    screen.clear()

    for x in range(len(magn[0])):
        for y in range(HEIGHT-1):
            if magn[0][x] > 5*(HEIGHT-y):
                screen.addch(y,x,ord('#'))

    screen.refresh()


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/magn',channels,update_magn)
        while not rospy.is_shutdown():
            plot()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    try:
        screen.clear()
        screen.addstr('\n GOODBYE \n\n', curses.color_pair(1))
        screen.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
