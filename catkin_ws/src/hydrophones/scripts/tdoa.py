#!/usr/bin/env python

# TODO: FIX NOT WORKING WITH TARGET OTHER THAN 30kHz
#       MOVE TO PEAK DETECTION INSTEAD OF THRESHOLD
#       MAKE SURE SIGNAL ALWAYS WITHIN BUFFERSIZE
#       IMPLEMENT BANDPASS FILTER
#       DISTINGUISH PRACTICE FROM COMPETITION

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param
import time

# PARAMETERS
try:
    BUFFERSIZE = param.get_buffersize()
    FINAL_BUFFERSIZE = param.get_final_buffersize()
    LIN_TO_MIC_OFFSET = param.get_line_in_offset()
    NUMBER_OF_MICS = param.get_number_of_mics()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    TARGET_FREQUENCY = param.get_target_frequency()
    PRACTICE = param.get_practice_pool_side_or_not()
    THRESHOLD = param.get_threshold()
except:
    print 'ROS NOT RUNNING'
    exit(1)

# USEFUL CONSTANTS
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
INTERPOLATION = 0.001

# VARIABLES
crunching = False
target = False
waiting = False
last_ping_time = time.time()
first_signal = channels()
second_signal = channels()
frequencies = freq()
dt = tdoa()

# NODE AND PUBLISHERS
rospy.init_node('tdoa')
tdoa_topic = rospy.Publisher('/hydrophones/tdoa',tdoa)
freq_topic = rospy.Publisher('/hydrophones/freq',freq)


def update_params():
    """ Updates target frequency """
    global TARGET_FREQUENCY, TARGET_INDEX, THRESHOLD
    TARGET_FREQUENCY = param.get_target_frequency()
    TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
    THRESHOLD = param.get_threshold()


def acquire_target():
    """ Searches for target frequency in first signal """
    global last_ping_time, target
    freq = [np.zeros(BUFFERSIZE/2+1,np.float)
            for i in range(NUMBER_OF_MICS)]

    # FFT
    freq[0] = np.fft.rfft(first_signal.channel_0)
    freq[1] = np.fft.rfft(first_signal.channel_1)
    freq[2] = np.fft.rfft(first_signal.channel_2)
    freq[3] = np.fft.rfft(first_signal.channel_3)

    # PUBLISH
    frequencies.channel_0.real = np.real(freq[0])
    frequencies.channel_0.imag = np.imag(freq[0])
    frequencies.channel_1.real = np.real(freq[1])
    frequencies.channel_1.imag = np.imag(freq[1])
    frequencies.channel_2.real = np.real(freq[2])
    frequencies.channel_2.imag = np.imag(freq[2])
    frequencies.channel_3.real = np.real(freq[3])
    frequencies.channel_3.imag = np.imag(freq[3])
    freq_topic.publish(frequencies)

    # DETERMINE IF TARGET FREQUENCY APPEARS
    for i in range(NUMBER_OF_MICS):
        magnitude = np.absolute(freq[i][TARGET_INDEX])
        if magnitude > THRESHOLD:
            # CHECK IF TARGET PING OR DUMMY PING
            current_time = time.time()
            if current_time - last_ping_time > 1:
                target = not PRACTICE
            else:
                target = PRACTICE

            last_ping_time = current_time

            return True

    return False


def parse(data):
    """ Deals with subscribed audio data """
    global crunching, waiting, first_signal, second_signal
    if not crunching:
        update_params()
        if not waiting:
            first_signal = data
            if acquire_target():
                waiting = True
        else:
            second_signal = data
            crunching = True
            gccphat()
            crunching = False
            waiting = False


def interpolate(x,s,u):
    """ Interpolates x with a sinc function """
    T = s[1] - s[0]
    sincM = np.tile(u, (len(s), 1)) - \
            np.tile(s[:, np.newaxis], (1, len(u)))
    y = np.dot(x, np.sinc(sincM/T))

    return y


def gccphat():
    """ Computes Time Difference of Arrival """
    # PARSE
    first_time = [first_signal.channel_0, first_signal.channel_1,
                  first_signal.channel_2, first_signal.channel_3]
    second_time = [second_signal.channel_0, second_signal.channel_1,
                   second_signal.channel_2, second_signal.channel_3]
    time = first_time + second_time

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
        end = min(index+5,FINAL_BUFFERSIZE)
        s = np.arange(begin,end)
        u = np.arange(begin,end,INTERPOLATION)
        phat_interp = interpolate(phat[begin:end],s,u)
        top = np.argmax(phat_interp)

        # TIME DIFFERENCE
        if index < FINAL_BUFFERSIZE/4:
            diff[i] = -u[top] / SAMPLING_FREQUENCY
        else:
            diff[i] = (FINAL_BUFFERSIZE - u[top]) / SAMPLING_FREQUENCY

    # PUBLISH
    dt.tdoa_1 = diff[1]
    dt.tdoa_2 = diff[2] - LIN_TO_MIC_OFFSET
    dt.tdoa_3 = diff[3] - LIN_TO_MIC_OFFSET
    dt.target = target
    tdoa_topic.publish(dt)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/audio',channels,parse)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
