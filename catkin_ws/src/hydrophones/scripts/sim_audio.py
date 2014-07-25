#!/usr/bin/env python

# TODO: 3 DIMENSIONS (RPY)

# IMPORTS
import numpy as np
import rospy
import roslib
import time
from scipy import signal as sp
from hydrophones.msg import *
from std_msgs.msg import Bool
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
audio_topic = rospy.Publisher('/hydrophones/audio', channels, tcp_nodelay=True, queue_size=0)
new_signal_notification = rospy.Publisher('/hydrophones/sim/target_active', Bool)
rate = rospy.Rate(SAMPLING_FREQUENCY/float(BUFFERSIZE))
signal = channels()

# VARIABLES
last_ping = time.time()
delta_time = 0.9
target = False


def create_signal(dt):
    """ Creates time shifted signal """
    global SNR, TARGET_FREQUENCY
    SNR = param.get_snr()
    TARGET_FREQUENCY = param.get_target_frequency()

    # SETUP
    delta = np.ceil(dt*SAMPLING_FREQUENCY)
    signal = np.random.normal(0,1,FINAL_BUFFERSIZE)
    ping = int(round(LENGTH_OF_PULSE*SAMPLING_FREQUENCY))

    # CREATE LINEAR CHIRP
    t = np.arange(ping) / float(SAMPLING_FREQUENCY)
    chirp = SNR * sp.chirp(t, TARGET_FREQUENCY - 200, t[ping/4], TARGET_FREQUENCY, phi=90)
    for i in range(ping):
        signal[i+delta+FINAL_BUFFERSIZE/4] += chirp[i]

    return signal


def compute_tdoa(x,y):
    """ Computes expected TDOA """
    times = []
    for channel in range(NUMBER_OF_MICS):
        magnitude = np.sqrt((POS[channel][0]-x)**2 + (POS[channel][1]-y)**2)
        times.append(magnitude/SPEED)

    for channel in range(1,NUMBER_OF_MICS):
        times[channel] -= times[0]
    times[0] = 0

    return times


def simulate():
    ''' Simulates microphone signals '''
    global delta_time, last_ping, target
    signal = channels()

    # CHECK IF TIME FOR PING
    dt = time.time() - last_ping
    if dt >= delta_time:
        # SETUP AND DETERMINE PINGER
        last_ping = time.time()
        new_signal_notification.publish(Bool(target))
        override = not param.get_switching()
        if target or override:
            (x,y) = param.get_simulation_target()
            if override:
                delta_time = 1.10
            else:
                delta_time = 0.90
        else:
            (x,y) = param.get_simulation_dummy()
            delta_time = 1.10
        if not override:
            target = not target

        # COMPUTE TDOA AND CREATE SIGNALS
        dt = compute_tdoa(x,y)
        channel_0 = create_signal(dt[0])
        channel_1 = create_signal(dt[1])
        channel_2 = create_signal(dt[2] + LIN_TO_MIC_OFFSET)
        channel_3 = create_signal(dt[3] + LIN_TO_MIC_OFFSET)

        # CREATE FIRST SIGNAL
        signal.channel_0 = channel_0[:BUFFERSIZE]
        signal.channel_1 = channel_1[:BUFFERSIZE]
        signal.channel_2 = channel_2[:BUFFERSIZE]
        signal.channel_3 = channel_3[:BUFFERSIZE]
        signal.stamp = rospy.Time.now()
        audio_topic.publish(signal)

        rate.sleep()

        # CREATE SECOND SIGNAL
        signal.channel_0 = channel_0[BUFFERSIZE:]
        signal.channel_1 = channel_1[BUFFERSIZE:]
        signal.channel_2 = channel_2[BUFFERSIZE:]
        signal.channel_3 = channel_3[BUFFERSIZE:]
        signal.stamp = rospy.Time.now()
        audio_topic.publish(signal)

    else:
        # CREATE NOISY SOUND OTHERWISE
        signal.channel_0 = np.random.normal(0,1,BUFFERSIZE)
        signal.channel_1 = np.random.normal(0,1,BUFFERSIZE)
        signal.channel_2 = np.random.normal(0,1,BUFFERSIZE)
        signal.channel_3 = np.random.normal(0,1,BUFFERSIZE)
        signal.stamp = rospy.Time.now()

        # CHECK OVERRIDE
        if param.get_continuous():
            audio_topic.publish(signal)

    rate.sleep()

if __name__ == '__main__':
    try:
        rospy.logwarn('SIMULATION RUNNING')
        while not rospy.is_shutdown():
            simulate()
    except rospy.ROSInterruptException:
        pass

