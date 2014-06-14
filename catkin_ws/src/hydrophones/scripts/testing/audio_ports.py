#!/usr/bin/env python

# IMPORTS
from ctypes import *
import pyaudio

# SET UP ERROR HANDLER TO FILTER OUT PYAUDIO MESSAGES
def py_error_handler(filename, line, function, err, fmt):
    pass
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int,
                               c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)

# SET UP PYAUDIO
audio = pyaudio.PyAudio()

# PRINT OUT RELEVANT INFORMATION
print '\nThe following audio inputs are available:\n'
for i in range(audio.get_device_count()):
    device = audio.get_device_info_by_index(i)
    if device['maxInputChannels'] > 0:
        print 'Index: %d' % device['index']
        print 'Name: %s' % device['name']
        print 'Channels: %d' % device['maxInputChannels']
        print ''
