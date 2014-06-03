#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
param.set_simulation_parameters()
NUMBER_OF_MICS = param.get_number_of_mics()
BUFFERSIZE = param.get_buffersize()
SAMPLING_FREQUENCY = param.get_sampling_frequency()
TARGET_FREQUENCY = param.get_target_frequency()
SPEED = param.get_speed()
POS = param.get_mic_positions()
LENGTH_OF_PULSE = param.get_pulse_length()
SNR = param.get_snr()

# SET UP NODE AND TOPIC
rospy.init_node('audio')
audio_topic = rospy.Publisher('/hydrophones/channels', channels)
signal = channels()
rate = rospy.Rate(0.5)

# VARIABLES
time = np.arange(BUFFERSIZE) / float(SAMPLING_FREQUENCY)


def create_signal(dt):
    """ Create time shifted signal """
    delta = np.ceil(dt*SAMPLING_FREQUENCY)
    signal = np.zeros(BUFFERSIZE,np.float32)
    for i in range(int(round(LENGTH_OF_PULSE*SAMPLING_FREQUENCY))):
        t = time[i+delta] - dt
        signal[i+delta+300] = SNR*np.sin(2*np.pi*TARGET_FREQUENCY*t)

    return signal


def compute_tdoa():
    """ Computes expected TDOA """
    (x,y) = param.get_simulation_solution()

    times = []
    for i in range(NUMBER_OF_MICS):
        magnitude = np.sqrt((POS[i][0]-x)**2 + (POS[i][1]-y)**2)
        times.append(magnitude/SPEED)

    for i in range(1,NUMBER_OF_MICS):
        times[i] -= times[0]
    times[0] = 0

    return times


def simulate():
    ''' Simulate microphone signals '''
    dt = compute_tdoa()
    signal = channels()

    signal.channel_0 = create_signal(dt[0])
    signal.channel_1 = create_signal(dt[1])
    signal.channel_2 = create_signal(dt[2])
    signal.channel_3 = create_signal(dt[3])

    audio_topic.publish(signal)


if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            simulate()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

