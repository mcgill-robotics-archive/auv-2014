#!/usr/bin/env python

# TODO: MAKE SURE LINE IN AND MIC WORK AS EXPECTED

# IMPORTS
from ctypes import *
import pyaudio
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
except:
    print 'ROS NOT RUNNING'
    exit(1)

# SET UP NODE AND TOPIC
rospy.init_node('audio')
audio_topic = rospy.Publisher('/hydrophones/audio', channels)
signal = channels()


# SET UP ERROR HANDLER TO FILTER OUT PYAUDIO MESSAGES
def py_error_handler(filename, line, function, err, fmt):
    pass

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int,
                               c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)


def setup():
    """ Sets up audio streams """
    global audio, mic, lin, inputs
    audio = pyaudio.PyAudio()
    mic = audio.open(format=pyaudio.paFloat32, channels=2,
                     rate=SAMPLING_FREQUENCY, input=True,
                     frames_per_buffer=BUFFERSIZE)
    lin = audio.open(format=pyaudio.paFloat32, channels=2,
                     rate=SAMPLING_FREQUENCY, input=True,
                     frames_per_buffer=BUFFERSIZE)
    inputs = {'mic':mic, 'lin':lin}


def read():
    """ Reads signal from microphones """
    stream = []
    for card in inputs:
        data = np.fromstring(inputs[card].read(BUFFERSIZE),
                             dtype=np.float32)
        stream.extend([data[::2],data[1::2]])

    signal.channel_0 = stream[0]
    signal.channel_1 = stream[1]
    signal.channel_2 = stream[2]
    signal.channel_3 = stream[3]

    audio_topic.publish(signal)


def close():
    """ Closes audio streams """
    audio.close(mic)
    audio.close(lin)
    audio.terminate()


if __name__ == '__main__':
    try:
        setup()
        while not rospy.is_shutdown():
            read()
    except rospy.ROSInterruptException:
        pass

    close()
