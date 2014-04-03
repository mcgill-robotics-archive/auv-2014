#!/usr/bin/env python

# IMPORTS
from ctypes import *
import pyaudio
import numpy as np

# VARIABLES
BUFFERSIZE = 512
NUMBER_OF_MICS = 2
SAMPLING_FREQUENCY = 48000  # IN Hz
TARGET_FREQUENCY = 1000     # IN Hz
RANGE = 2                   # HOW FAR ON EACH SIDE TO LOOK
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
SPEED = 343                 # SPEED IN AIR IN m/s


# SET UP ERROR HANDLER TO FILTER OUT PYAUDIO MESSAGES
def py_error_handler(filename, line, function, err, fmt):
    pass

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)


# MIC OBJECT
class mic(object):
    __slots__ = ['name', 'time', 'frequency', 'magnitude', 'angle', 'distance']

    def __init__(self, quadrant):
        self.reset(quadrant)

    def reset(self, quadrant):
        self.name = quadrant + 1
        self.time = []
        self.frequency = []
        self.magnitude = []
        self.angle = []
        self.distance = np.zeros(BUFFERSIZE/2 + 1, np.float);

    def fft(self):
        self.frequency = np.fft.rfft(self.time)

    def decibel(self):
        self.magnitude = np.absolute(self.frequency)

    def phase(self):
        self.angle = np.degrees(np.arctan2(self.frequency.imag, self.frequency.real)) - 180

    def distances(self):
        frequency = np.multiply(FREQUENCY_PER_INDEX, range(1, BUFFERSIZE / 2 + 1))
        self.distance[1:] = self.angle[1:] / 360. / 2 / frequency * SPEED


# CREATE MIC OBJECTS
mics = [mic(i) for i in range(NUMBER_OF_MICS)]

# SET UP SOUND INPUT
audio = pyaudio.PyAudio()
mic_in = audio.open(format=pyaudio.paFloat32, channels=2, rate=SAMPLING_FREQUENCY,
                    input=True, frames_per_buffer=BUFFERSIZE)
# line_in = audio.open(format=pyaudio.paFloat32, channels=2, rate=SAMPLING_FREQUENCY,
                     # input=True, frames_per_buffer=BUFFERSIZE)


# READ INPUT
def read():
    # READ AND STORE DATA
    data_mic = np.fromstring(mic_in.read(BUFFERSIZE), dtype=np.float32)
    # data_lin = np.fromstring(line_in.read(BUFFERSIZE), dtype=np.float32)
    mics[0].time = data_mic[::2]
    mics[1].time = data_mic[1::2]
    # mics[2].time = data_lin[::2]
    # mics[3].time = data_lin[1::2]


# FFT AND MAGNITUDE
def process():
    for i in range(NUMBER_OF_MICS):
        mics[i].fft()
        mics[i].decibel()
        mics[i].phase()

    for i in range(1, NUMBER_OF_MICS):
        for j in range(BUFFERSIZE/2+1):
            mics[i].angle[j] -= mics[0].angle[j]
            mics[0].angle[j] = 0
        mics[i].distances()

# LOOK AT RELEVANT FREQUENCY
def analyze():
    print '%s\t%s\t%s\t%s\t\t%s\n' % ('MIC', 'FREQUENCY', 'MAGNITUDE', 'PHASE', 'DISTANCE')

    for i in range(NUMBER_OF_MICS):
        for j in range(INDEX - RANGE, INDEX + RANGE + 1):
            print '%1d\t  %4d Hz\t%+3.2f\t\t%+3.2f\t\t%+3.2f' % (i, j * FREQUENCY_PER_INDEX,
                                                                 mics[i].magnitude[j],
                                                                 mics[i].angle[j],
                                                                 mics[i].distance[j])
        print ""


# FIND MAX
def maximize():
    for i in range(NUMBER_OF_MICS):
        max = np.argmax(mics[i].magnitude)
        print '%s %d\t  %4d Hz\t%+3.2f\t\t%+3.2f\t\t%+3.2f' % ('MAX', i, max * FREQUENCY_PER_INDEX,
                                                               mics[i].magnitude[max],
                                                               mics[i].angle[max],
                                                               mics[i].distance[max])

# SHOW MAX
read()
process()
analyze()
maximize()

while True:
    read()
    process()
    maximize()
