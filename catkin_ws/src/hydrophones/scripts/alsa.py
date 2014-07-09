#!/usr/bin/env python

# TODO: MAKE SURE LINE IN AND MIC WORK AS EXPECTED

# IMPORTS
import alsaaudio
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
try:
    BUFFERSIZE = param.get_buffersize()
    NUMBER_OF_MICS = param.get_number_of_mics()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    PERIOD = 512
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('audio')
audio_topic = rospy.Publisher('/hydrophones/audio', channels)
signal = channels()


def setup():
    """ Sets up audio streams """
    global inputs
    mic = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card='hw:0,0')
    lin = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card='hw:0,1')
    inputs = {'mic':mic, 'lin':lin}

    for card in inputs:
        inputs[card].setchannels(2)
        inputs[card].setrate(SAMPLING_FREQUENCY)
        inputs[card].setperiodsize(PERIOD)
        inputs[card].setformat(alsaaudio.PCM_FORMAT_FLOAT_LE)


def read():
    """ Reads and parses audio streams """
    stream = [[] for channel in range(NUMBER_OF_MICS)]
    for counter in range(BUFFERSIZE/PERIOD):
        current_stream = []
        for card in inputs:
            data = np.fromstring(inputs[card].read()[1],dtype=np.float32)
            current_stream.extend([data[::2],data[1::2]])
        for channel in range(NUMBER_OF_MICS):
            stream[channel].extend(current_stream[channel])

    signal.channel_0 = stream[0]
    signal.channel_1 = stream[1]
    signal.channel_2 = stream[2]
    signal.channel_3 = stream[3]

    audio_topic.publish(signal)


def close():
    """ Closes audio streams """
    for card in inputs:
        inputs[card].close()


if __name__ == '__main__':
    try:
        setup()
        while not rospy.is_shutdown():
            read()
    except rospy.ROSInterruptException:
        pass

    close()
