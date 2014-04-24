#!/usr/bin/env python

### IMPORTS
from ctypes import *
import pyaudio
import numpy as np
<<<<<<< HEAD
import curses
import os
import time
import sys

### VARIABLES
NUMBER_OF_MICS = 2          # MICROPHONES CONNECTED
BUFFERSIZE = 256            # BUFFERSIZE FOR FFT
SAMPLING_FREQUENCY = 48000  # SAMPLING FREQUENCY            Hz
TARGET_FREQUENCY = 1000     # FREQUENCY TO ACT UPON         Hz
FREQUENCY_RANGE = 100       # RANGE OFF TARGET TO MONITOR   Hz
SPEED = 343                 # SPEED OF SOUND IN AIR         m/s

### USEFUL CONSTANTS
=======

# VARIABLES
BUFFERSIZE = 512
NUMBER_OF_MICS = 2
SAMPLING_FREQUENCY = 48000  # IN Hz
TARGET_FREQUENCY = 1000     # IN Hz
RANGE = 2                   # HOW FAR ON EACH SIDE TO LOOK
>>>>>>> FETCH_HEAD
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TIME_PER_INDEX = 1 / SAMPLING_FREQUENCY
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
RANGE = int(round(FREQUENCY_RANGE / FREQUENCY_PER_INDEX))
RFFT_LENGTH = BUFFERSIZE / 2 + 1
LABELS = ['I', 'II', 'III', 'IV']

### SET UP CURSES
curses.initscr()
curses.echo()
height = NUMBER_OF_MICS * (2 * RANGE + 4) + 8
screen = curses.newwin(height, 100)
screen.clear()
curses.start_color()
curses.use_default_colors()
curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_RED)   # WARNING
curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)  # SUCCESS
curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE) # HEADING


class mic(object):
<<<<<<< HEAD
    ''' Microphone class '''
    __slots__ = ['label', 'time', 'frequency', 'magnitude',
                 'phase', 'difference', 'distance']

    def __init__(self, i):
        ''' Initializes mic object '''
        self.reset(i)

    def reset(self, i):
        ''' Resets mic object '''
        self.label = LABELS[i]
        self.time = [0. for x in range(BUFFERSIZE)]
        self.frequency = [0. for x in range(RFFT_LENGTH)]
        self.magnitude = [0. for x in range(RFFT_LENGTH)]
        self.phase = [0. for x in range(RFFT_LENGTH)]
        self.difference = 0
        self.distance = 0
=======
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
>>>>>>> FETCH_HEAD

    def fft(self):
        ''' FFTs time domain '''
        self.frequency = np.fft.rfft(self.time)

    def update_magnitude(self):
        ''' Computes magnitude '''
        self.magnitude = np.absolute(self.frequency)

    def update_phase(self):
        ''' Computes phase in degrees '''
        # self.phase = np.degrees(np.arctan2(self.frequency.imag, self.frequency.real)) - 180
        pass

    def compute_phase_difference(self):
        ''' Computes phase difference in seconds '''
        if self.label == 'I':
            self.difference = 0
        else:
            gcc = np.multiply(np.conj(mics[0].frequency), self.frequency)
            phat = np.fft.ifft(np.divide(gcc, np.absolute(gcc)))
            self.difference = np.argmax(phat) * TIME_PER_INDEX

    def update_distance(self):
        ''' Computes distance in meters '''
        self.distance = self.difference * SPEED


### CREATE MIC OBJECTS
mics = [mic(i) for i in range(NUMBER_OF_MICS)]


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
# lin = audio.open(format=pyaudio.paFloat32, channels=2,
#                  rate=SAMPLING_FREQUENCY, input=True,
#                  frames_per_buffer=BUFFERSIZE)


def read():
    ''' Reads time domain from microphones and updates objects '''
    data_mic = np.fromstring(mic.read(BUFFERSIZE), dtype=np.float32)
    # data_lin = np.fromstring(lin.read(BUFFERSIZE), dtype=np.float32)

    mics[0].time = data_mic[::2]
    mics[1].time = data_mic[1::2]
    # mics[2].time = data_lin[::2]
    # mics[3].time = data_lin[1::2]


def process():
    ''' FFTs the time domain and computes its magnitude and phase '''
    for i in range(NUMBER_OF_MICS):
        mics[i].fft()
        mics[i].update_magnitude()
        mics[i].update_phase()

<<<<<<< HEAD
    for i in range(NUMBER_OF_MICS):
        for j in range(RFFT_LENGTH):
            mics[i].compute_phase_difference()

        mics[i].update_distance()


=======
    for i in range(1, NUMBER_OF_MICS):
        for j in range(BUFFERSIZE/2+1):
            mics[i].angle[j] -= mics[0].angle[j]
            mics[0].angle[j] = 0
        mics[i].distances()

# LOOK AT RELEVANT FREQUENCY
>>>>>>> FETCH_HEAD
def analyze():
    ''' Monitors target frequency and prints table '''
    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    screen.addstr(0, 0, target)

    header = '\n%s\t%s\t%s\t%s\t\n\n' % (' MIC', 'FREQUENCY', 'MAGNITUDE', 'TDOA')
    screen.addstr(header, curses.color_pair(3))

    for i in range(NUMBER_OF_MICS):
        for j in range(TARGET_INDEX - RANGE, TARGET_INDEX + RANGE + 1):
            value = '%s\t  %4d Hz\t%+4.2f\t\t%+4.2f\n' % (' ' + mics[i].label,
                                                          j * FREQUENCY_PER_INDEX,
                                                          mics[i].magnitude[j],
                                                          mics[i].difference)
            screen.addstr(value)

        screen.addstr('\n')


def maximize():
    ''' Finds max frequency and prints table '''
    screen.addstr(' MAX\t\t\t\t\t\t\n\n', curses.color_pair(3))
    for i in range(NUMBER_OF_MICS):
        max = np.argmax(mics[i].magnitude)
<<<<<<< HEAD

        state = 0
        if max >= TARGET_INDEX - RANGE and max <= TARGET_INDEX + RANGE:
            state = 2

        value = '%s\t  %4d Hz\t%+4.2f\t\t%+4.2f\t\n' % (' ' + mics[i].label,
                                                        max * FREQUENCY_PER_INDEX,
                                                        mics[i].magnitude[max],
                                                        mics[i].difference)

        screen.addstr(value, curses.color_pair(state))




def close():
    ''' Closes audio streams '''
    audio.close(mic)
    # audio.close(lin)


if __name__ == '__main__':
    ### MAIN
    while True:
        try:
            read()
            process()
            analyze()
            maximize()
            screen.refresh()

        except KeyboardInterrupt:
            break

    ### QUIT PEACEFULLY
    try:
        close()
        screen.addstr('\n GOODBYE \n\n', curses.color_pair(1))
        screen.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        os.system('clear')
        exit(0)
=======
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
>>>>>>> FETCH_HEAD
