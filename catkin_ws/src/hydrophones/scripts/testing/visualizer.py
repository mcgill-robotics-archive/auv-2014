#!/usr/bin/env python

### IMPORTS
from ctypes import *
import pyaudio
import numpy as np
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
curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_RED)    # WARNING
curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_BLUE)   # SUCCESS
curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_WHITE)  # HEADING


class mic(object):
    ''' Microphone class '''
    __slots__ = ['label', 'time', 'frequency', 'magnitude',
                 'difference', 'distance']

    def __init__(self, quadrant):
        ''' Initializes mic object '''
        self.reset(quadrant)

    def reset(self, quadrant):
        ''' Resets mic object '''
        self.label = LABELS[quadrant]
        self.time = np.zeros(BUFFERSIZE, np.float)
        self.frequency = np.zeros(RFFT_LENGTH, np.float)
        self.magnitude = np.zeros(RFFT_LENGTH, np.float)
        self.difference = 0
        self.distance = 0

    def compute_fft(self):
        ''' FFTs time domain '''
        self.frequency = np.fft.rfft(self.time)

    def compute_magnitude(self):
        ''' Computes magnitude '''
        self.magnitude = np.absolute(self.frequency)

    def compute_phase_difference(self):
        ''' Computes phase difference in microseconds '''
        if self.label == LABELS[0]:
            self.difference = 0
        else:
            gcc = np.multiply(np.conj(mics[0].frequency), self.frequency)
            phat = np.fft.ifft(np.divide(gcc, np.absolute(gcc)))
            self.difference = np.argmax(phat) * TIME_PER_INDEX * 1e6

    def compute_distance(self):
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
        mics[i].compute_fft()
        mics[i].compute_magnitude()
        mics[i].compute_phase_difference()
        mics[i].compute_distance()


# LOOK AT RELEVANT FREQUENCY
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
