#!/usr/bin/env python

# IMPORTS
import alsaaudio
import numpy as np
import rospy
import roslib
import os
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
    card_number = os.popen("arecord -l | grep 'Intel'").read().split()[1][0]
    cards = ['plughw:%s,0' % card_number, 'plughw:%s,2' % card_number]

    mic = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card=cards[0])
    lin = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card=cards[1])
    inputs = {'mic':mic, 'lin':lin}

    for card in inputs:
        print ""
        rospy.logwarn("%s cardname:   %s", card.upper(), inputs[card].cardname())
        rospy.logwarn("%s channels:   %d", card.upper(), inputs[card].setchannels(2))
        rospy.logwarn("%s sampling:   %d", card.upper(), inputs[card].setrate(SAMPLING_FREQUENCY))
        rospy.logwarn("%s period:     %d", card.upper(), inputs[card].setperiodsize(PERIOD))
        rospy.logwarn("%s format:     %d", card.upper(), inputs[card].setformat(alsaaudio.PCM_FORMAT_FLOAT_LE))

    print ""


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
    signal.stamp = rospy.Time.now()

    audio_topic.publish(signal)


def close():
    """ Closes audio streams """
    for card in inputs:
        inputs[card].close()


if __name__ == '__main__':
    try:
        try:
            setup()

        except:
            rospy.logfatal('AUDIO STREAMS COULD NOT BE OPENED')
            exit(1)

        while not rospy.is_shutdown():
            read()

    except rospy.ROSInterruptException:
        pass

    close()
