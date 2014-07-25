#!/usr/bin/env python

# TODO: MOVE TO PEAK DETECTION INSTEAD OF THRESHOLD
#       IMPLEMENT BANDPASS FILTER
#       DOUBLE CHECK TDOA SIGNS AND INTERPOLATION!!!

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

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

# NODE AND PUBLISHERS
rospy.init_node('tdoa')
tdoa_topic = rospy.Publisher('/hydrophones/tdoa',tdoa,tcp_nodelay=True,queue_size=0)
freq_topic = rospy.Publisher('/hydrophones/freq',freq)

# VARIABLES
crunching = False
target = False
waiting = False
last_ping_time = rospy.Time.now()
first_signal = channels()
second_signal = channels()
frequencies = freq()
dt = tdoa()

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
    for channel in range(NUMBER_OF_MICS):
        magnitude = 20*np.log10(np.abs(freq[channel][TARGET_INDEX]))
        if magnitude > THRESHOLD:
            # CHECK IF TARGET PING OR DUMMY PING
            current_time = first_signal.stamp
            delta_time = current_time - last_ping_time
            if delta_time > rospy.Duration(1):
                target = not PRACTICE
            else:
                target = PRACTICE
            rospy.logwarn("Triggered on %s at %3.2f dB after %1.3f s",
                          'TARGET' if target else 'DUMMY ', magnitude, delta_time.to_sec())
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
            waiting = False
            gccphat()
            crunching = False


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
    time = [[] for channel in range(NUMBER_OF_MICS)]
    for channel in range(NUMBER_OF_MICS):
        time[channel] = first_time[channel] + second_time[channel]

    # FFT
    freq = [[] for channel in range(NUMBER_OF_MICS)]
    for channel in range(NUMBER_OF_MICS):
        freq[channel] = np.fft.fft(time[channel])

    # COMPUTE TDOA
    diff = [0 for channel in range(NUMBER_OF_MICS)]
    for channel in range(1,NUMBER_OF_MICS):
        # ORDINARY GCC-PHAT
        gcc = np.multiply(freq[0],np.conj(freq[channel]))
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
        # DOUBLE CHECK THIS!!!
        if index < FINAL_BUFFERSIZE/4:
            diff[channel] = -u[top] / SAMPLING_FREQUENCY
        else:
            diff[channel] = (FINAL_BUFFERSIZE - u[top]) / SAMPLING_FREQUENCY

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
