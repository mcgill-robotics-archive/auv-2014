#!/usr/bin/env python

# TODO: TEST WITH KNOWN TIME DIFFERENCES
#       IMPLEMENT BANDPASS FILTER
#       DISTINGUISH PRACTICE FROM COMPETITION

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
INTERPOLATION = 0.001
BUFFERSIZE = param.get_buffersize()
NUMBER_OF_MICS = param.get_number_of_mics()
SAMPLING_FREQUENCY = param.get_sampling_frequency()
TARGET_FREQUENCY = param.get_target_frequency()

# USEFUL CONSTANTS
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
THRESHOLD = 0.1

# VARIABLES
crunching = False
signal = channels()
dt = tdoa()


def acquire_target():
    """ Searches for target frequency in signal """
    freq = [np.zeros(BUFFERSIZE/2+1,np.float)
            for i in range(NUMBER_OF_MICS)]

    freq[0] = np.fft.rfft(signal.channel_0)
    freq[1] = np.fft.rfft(signal.channel_1)
    freq[2] = np.fft.rfft(signal.channel_2)
    freq[3] = np.fft.rfft(signal.channel_3)

    for i in range(NUMBER_OF_MICS):
        magnitude = np.absolute(freq[i][TARGET_INDEX])
        if magnitude > THRESHOLD:
            return True

    return False


def parse(data):
    """ Deals with subscribed audio data """
    global crunching, signal
    if not crunching:
        signal = data
        if acquire_target():
            crunching = True
            gccphat()
            crunching = False


def interpolate(x,s,u):
    """ Interpolate x with a sinc function """
    T = s[1] - s[0]
    sincM = np.tile(u, (len(s), 1)) - \
            np.tile(s[:, np.newaxis], (1, len(u)))
    y = np.dot(x, np.sinc(sincM/T))

    return y


def gccphat():
    """ Compute Time Difference of Arrival """
    # PARSE
    time = [signal.channel_0, signal.channel_1,
            signal.channel_2, signal.channel_3]

    # FFT
    freq = [[] for i in range(NUMBER_OF_MICS)]
    for i in range(NUMBER_OF_MICS):
        freq[i] = np.fft.fft(time[i])

    # COMPUTE TDOA
    diff = [0 for i in range(NUMBER_OF_MICS)]
    for i in range(1,NUMBER_OF_MICS):
        # ORDINARY GCC-PHAT
        gcc = np.multiply(freq[0],np.conj(freq[i]))
        phat = np.fft.ifft(np.divide(gcc,np.absolute(gcc)))
        phat = np.absolute(phat)
        index = np.argmax(phat)

        # INTERPOLATION
        begin = max(0,index-5)
        end = min(index+5,BUFFERSIZE)
        s = np.arange(begin,end)
        u = np.arange(begin,end,INTERPOLATION)
        phat_interp = interpolate(phat[begin:end],s,u)
        top = np.argmax(phat_interp)

        # TIME DIFFERENCE
        if index < (BUFFERSIZE)/2:
            diff[i] = (index + u[top]) / SAMPLING_FREQUENCY
        else:
            diff[i] = (index + u[top] - BUFFERSIZE) / \
                      SAMPLING_FREQUENCY

    # PUBLISH
    dt.tdoa_1 = diff[1]
    dt.tdoa_2 = diff[2]
    dt.tdoa_3 = diff[3]
    tdoa_topic.publish(dt)


if __name__ == '__main__':
    try:
        rospy.init_node('tdoa')
        tdoa_topic = rospy.Publisher('time_difference',tdoa)
        rospy.Subscriber('/hydrophones/channels',channels,parse)
        while not rospy.is_shutdown():
            pass
    except rospy.ROSInterruptException:
        pass
