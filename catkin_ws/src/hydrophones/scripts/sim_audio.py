#!/usr/bin/env python

# TODO: ADD NOISE
#       USE SNR CORRECTLY
#       3 DIMENSIONS
#       SIMULATE BOTH PRACTICE AND COMPETITION

# IMPORTS
import numpy as np
import rospy
import roslib
from scipy import signal as sp
from hydrophones.msg import *
import param

# PARAMETERS
try:
    param.set_simulation_parameters()
    BUFFERSIZE = param.get_buffersize()
    FINAL_BUFFERSIZE = param.get_final_buffersize()
    LIN_TO_MIC_OFFSET = param.get_line_in_offset()
    LENGTH_OF_PULSE = param.get_pulse_length()
    NUMBER_OF_MICS = param.get_number_of_mics()
    POS = param.get_mic_positions()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    SPEED = param.get_speed()
    SNR = param.get_snr()
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('audio')
audio_topic = rospy.Publisher('/hydrophones/audio', channels)
signal = channels()
rate = rospy.Rate(0.5)

# VARIABLES
time = np.arange(FINAL_BUFFERSIZE) / float(SAMPLING_FREQUENCY)


def create_signal(dt):
    """ Creates time shifted signal """
    global SNR, TARGET_FREQUENCY
    SNR = param.get_snr()
    TARGET_FREQUENCY = param.get_target_frequency()

    delta = np.ceil(dt*SAMPLING_FREQUENCY)
    signal = np.random.normal(0,1,FINAL_BUFFERSIZE)
    ping = int(round(LENGTH_OF_PULSE*SAMPLING_FREQUENCY))

    t = np.arange(ping)/float(SAMPLING_FREQUENCY)
    chirp = SNR * sp.chirp(t, TARGET_FREQUENCY - 200, t[ping/4], TARGET_FREQUENCY, phi=90)
    for i in range(ping):
        signal[i+delta+FINAL_BUFFERSIZE/4] += chirp[i]

    return signal


def compute_tdoa():
    """ Computes expected TDOA """
    (x,y) = param.get_simulation_target()

    times = []
    for i in range(NUMBER_OF_MICS):
        magnitude = np.sqrt((POS[i][0]-x)**2 + (POS[i][1]-y)**2)
        times.append(magnitude/SPEED)

    for i in range(1,NUMBER_OF_MICS):
        times[i] -= times[0]
    times[0] = 0

    return times


def simulate():
    ''' Simulates microphone signals '''
    dt = compute_tdoa()
    signal = channels()

    channel_0 = create_signal(dt[0])
    channel_1 = create_signal(dt[1])
    channel_2 = create_signal(dt[2] + LIN_TO_MIC_OFFSET)
    channel_3 = create_signal(dt[3] + LIN_TO_MIC_OFFSET)

    signal.channel_0 = channel_0[:BUFFERSIZE]
    signal.channel_1 = channel_1[:BUFFERSIZE]
    signal.channel_2 = channel_2[:BUFFERSIZE]
    signal.channel_3 = channel_3[:BUFFERSIZE]

    audio_topic.publish(signal)

    signal.channel_0 = channel_0[BUFFERSIZE:]
    signal.channel_1 = channel_1[BUFFERSIZE:]
    signal.channel_2 = channel_2[BUFFERSIZE:]
    signal.channel_3 = channel_3[BUFFERSIZE:]

    audio_topic.publish(signal)


if __name__ == '__main__':
    try:
        rospy.logwarn('SIMULATION RUNNING')
        while not rospy.is_shutdown():
            simulate()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

