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
    LENGTH_OF_PULSE = param.get_pulse_length()
    LINEAR_CHIRP = param.get_linear_chirp_or_not()
    NUMBER_OF_MICS = param.get_number_of_mics()
    POS = param.get_mic_positions()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    SPEED = param.get_speed()
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('audio')
audio_topic = rospy.Publisher('/hydrophones/audio', channels)
signal = channels()
rate = rospy.Rate(0.5)

# VARIABLES
time = np.arange(BUFFERSIZE) / float(SAMPLING_FREQUENCY)


def create_signal(dt):
    """ Creates time shifted signal """
    global SNR, TARGET_FREQUENCY, LINEAR_CHIRP
    SNR = param.get_snr()
    TARGET_FREQUENCY = param.get_target_frequency()
    LINEAR_CHIRP = param.get_linear_chirp_or_not()

    delta = np.ceil(dt*SAMPLING_FREQUENCY)
    signal = np.zeros(BUFFERSIZE,np.float32)
    ping = int(round(LENGTH_OF_PULSE*SAMPLING_FREQUENCY))

    t = np.arange(ping)/float(SAMPLING_FREQUENCY)
    chirp = sp.chirp(t, TARGET_FREQUENCY - 200, t[ping/4], TARGET_FREQUENCY, phi=90)
    for i in range(ping):
        signal[i+delta+BUFFERSIZE/2] = chirp[i]

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

    signal.channel_0 = create_signal(dt[0])
    signal.channel_1 = create_signal(dt[1])
    signal.channel_2 = create_signal(dt[2])
    signal.channel_3 = create_signal(dt[3])

    audio_topic.publish(signal)


if __name__ == '__main__':
    try:
        rospy.logwarn('SIMULATION RUNNING')
        while not rospy.is_shutdown():
            simulate()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

