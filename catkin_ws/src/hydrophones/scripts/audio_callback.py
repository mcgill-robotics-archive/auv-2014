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
mic_stream = [[],[]]
lin_stream = [[],[]]
mic_flag = False
lin_flag = False

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
                     frames_per_buffer=BUFFERSIZE,
                     stream_callback=mic_callback)
    lin = audio.open(format=pyaudio.paFloat32, channels=2,
                     rate=SAMPLING_FREQUENCY, input=True,
                     frames_per_buffer=BUFFERSIZE,
                     stream_callback=lin_callback)
    inputs = {'mic':mic, 'lin':lin}


def mic_callback(in_data, frame_count, time_info, status):
    """ Callback for MIC input """
    global mic_delay, mic_stream, mic_flag

    data = np.fromstring(in_data, dtype=np.float32)
    mic_stream[0].extend(data[::2])
    mic_stream[1].extend(data[1::2])
    mic_delay = time_info['input_buffer_adc_time']
    mic_flag = True

    return (None, pyaudio.paContinue)


def lin_callback(in_data, frame_count, time_info, status):
    """ Callback for LINE IN input """
    global lin_delay, lin_stream, lin_flag

    data = np.fromstring(in_data, dtype=np.float32)
    lin_stream[0].extend(data[::2])
    lin_stream[1].extend(data[1::2])
    lin_delay = time_info['input_buffer_adc_time']
    lin_flag = True

    return (None, pyaudio.paContinue)


def publish():
    """ Publishes signal from microphones """
    global mic_stream, lin_stream, mic_flag, lin_flag, signal
    if mic_flag and lin_flag:
        signal.channel_0 = mic_stream[0][:BUFFERSIZE]
        signal.channel_1 = mic_stream[1][:BUFFERSIZE]
        signal.channel_2 = lin_stream[0][:BUFFERSIZE]
        signal.channel_3 = lin_stream[1][:BUFFERSIZE]
        audio_topic.publish(signal)
        mic_flag = False
        lin_flag = False
        # for i in range(BUFFERSIZE):
        #     mic_stream[0].pop(0)
        #     mic_stream[1].pop(0)
        #     lin_stream[0].pop(0)
        #     lin_stream[1].pop(0)


def close():
    """ Closes audio streams """
    audio.close(mic)
    audio.close(lin)
    audio.terminate()


if __name__ == '__main__':
    try:
        setup()
        while not rospy.is_shutdown():
            publish()
    except rospy.ROSInterruptException:
        pass
    close()
