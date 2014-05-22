#!/usr/bin/env python

### IMPORTS
from ctypes import *
import pyaudio
import numpy as np

### SET UP ERROR HANDLER TO FILTER OUT PYAUDIO MESSAGES
def py_error_handler(filename, line, function, err, fmt):
    pass

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)

### SET UP SOUND INPUT
audio = pyaudio.PyAudio()
mic = audio.open(format=pyaudio.paFloat32, channels=2,
                 rate=SAMPLING_FREQUENCY, input=True,
                 frames_per_buffer=BUFFERSIZE)
lin = audio.open(format=pyaudio.paFloat32, channels=2,
                 rate=SAMPLING_FREQUENCY, input=True,
                 frames_per_buffer=BUFFERSIZE)


def read():
    ''' Reads time domain from microphones and updates objects '''
    data_mic = np.fromstring(mic.read(BUFFERSIZE), dtype=np.float32)
    data_lin = np.fromstring(lin.read(BUFFERSIZE), dtype=np.float32)

    mics[0].time = data_mic[::2]
    mics[1].time = data_mic[1::2]
    mics[2].time = data_lin[::2]
    mics[3].time = data_lin[1::2]
