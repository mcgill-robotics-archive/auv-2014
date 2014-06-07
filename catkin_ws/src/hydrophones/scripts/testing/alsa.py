#!/usr/bin/env python

# IMPORTS
import alsaaudio
import numpy as np

# VARIABLES
inputs = {}


def setup():
    """ Sets up audio streams """
    global inputs
    mic = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card='default')
    lin = alsaaudio.PCM(type=alsaaudio.PCM_CAPTURE,card='default')
    inputs = {'mic':mic, 'lin':lin}

    for card in inputs:
        inputs[card].setchannels(2)
        inputs[card].setrate(192000)
        inputs[card].setperiodsize(512)
        inputs[card].setformat(alsaaudio.PCM_FORMAT_FLOAT_LE)


def read():
    """ Reads and parses audio streams """
    stream = []
    for card in inputs:
        data = np.fromstring(inputs[card].read()[1],dtype=np.float32)
        stream.extend([data[::2],data[1::2]])

    return stream


def close():
    """ Closes audio streams """
    for card in inputs:
        inputs[card].close()


if __name__ == '__main__':
    setup()
    data = read()
    close()


# FOR DEBUGGING PURPOSES
print alsaaudio.cards()
print len(data[0]), len(data[1]), len(data[2]), len(data[3])
print max(data[0]), max(data[1]), max(data[2]), max(data[3])
