#!/usr/bin/env python

# IMPORTS
import rospy
import roslib
from hydrophones.msg import *
import param
import numpy as np
import matplotlib.pyplot as plt

# PARAMETERS
NUMBER_OF_MICS = param.get_number_of_mics()
BUFFERSIZE = param.get_buffersize()
SAMPLING_FREQUENCY = param.get_sampling_frequency()
TARGET_FREQUENCY = param.get_target_frequency()

# SET UP NODE
rospy.init_node('visualizer')
times = [t for t in range(BUFFERSIZE)]
plt.ion()


def plot_signal(signal):
    global times

    plt.subplot(411)
    plt.cla()
    plt.plot(times, signal.channel_0, 'r.-')

    plt.subplot(412)
    plt.cla()
    plt.plot(times, signal.channel_1, 'g.-')

    plt.subplot(413)
    plt.cla()
    plt.plot(times, signal.channel_2, 'b.-')

    plt.subplot(414)
    plt.cla()
    plt.plot(times, signal.channel_3, 'y.-')

    plt.draw()

    for t in range(BUFFERSIZE):
        times[t] += BUFFERSIZE


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/audio',channels,plot_signal)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
