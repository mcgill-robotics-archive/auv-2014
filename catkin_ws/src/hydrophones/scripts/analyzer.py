#!/usr/bin/env python

# IMPORTS
import numpy as np
import rospy
import roslib
from hydrophones.msg import *
import param

# PARAMETERS
try:
    NUMBER_OF_MICS = param.get_number_of_mics()
    BUFFERSIZE = param.get_buffersize()
    SAMPLING_FREQUENCY = param.get_sampling_frequency()
    FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('analyzer')
magn_topic = rospy.Publisher('/hydrophones/magn',channels, tcp_nodelay=True, queue_size=0)
peak_topic = rospy.Publisher('/hydrophones/peak',peaks, tcp_nodelay=True, queue_size=0)
peak_magn_topic = rospy.Publisher('/hydrophones/peak_magn',peaks, tcp_nodelay=True, queue_size=0)
magnitudes = channels()
peak_magn = peaks()
peaks = peaks()


def analyze_channel(channel):
    """ Computes magnitudes and finds peaks """
    np.seterr(divide='ignore')
    magn = 20*np.log10(np.abs(channel))
    peak = np.argmax(magn) * FREQUENCY_PER_INDEX

    return magn, peak


def analyze(frequencies):
    """ Analyzes all channels for magnitudes and peaks """
    # PARSE
    freq = [[] for channel in range(NUMBER_OF_MICS)]
    freq[0] = 1j * np.array(frequencies.channel_0.imag)
    freq[0] += frequencies.channel_0.real
    freq[1] = 1j * np.array(frequencies.channel_1.imag)
    freq[1] += frequencies.channel_1.real
    freq[2] = 1j * np.array(frequencies.channel_2.imag)
    freq[2] += frequencies.channel_2.real
    freq[3] = 1j * np.array(frequencies.channel_3.imag)
    freq[3] += frequencies.channel_3.real

    # ANALYZE
    magn = [[] for channel in range(NUMBER_OF_MICS)]
    peak = np.zeros(NUMBER_OF_MICS)
    for channel in range(NUMBER_OF_MICS):
        magn[channel], peak[channel] = analyze_channel(freq[channel])

    # PUBLISH MAGNITUDES
    magnitudes.channel_0 = magn[0]
    magnitudes.channel_1 = magn[1]
    magnitudes.channel_2 = magn[2]
    magnitudes.channel_3 = magn[3]
    magn_topic.publish(magnitudes)

    # PUBLISH PEAKS
    peaks.channel_0 = peak[0]
    peaks.channel_1 = peak[1]
    peaks.channel_2 = peak[2]
    peaks.channel_3 = peak[3]
    peak_topic.publish(peaks)

    # PUBLISH PEAK MAGNITUDES
    peak_magn.channel_0 = magn[0][int(peak[0]/FREQUENCY_PER_INDEX)]
    peak_magn.channel_1 = magn[1][int(peak[1]/FREQUENCY_PER_INDEX)]
    peak_magn.channel_2 = magn[2][int(peak[2]/FREQUENCY_PER_INDEX)]
    peak_magn.channel_3 = magn[3][int(peak[3]/FREQUENCY_PER_INDEX)]
    peak_magn_topic.publish(peak_magn)


if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/freq',freq,analyze)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
