#!/usr/bin/env python

# IMPORTS
from ctypes import *
import pyaudio
import numpy as np
import curses
import os
import time
import sys

# VARIABLES
NUMBER_OF_MICS = 4          # MICROPHONES CONNECTED
BUFFERSIZE = 1024           # BUFFERSIZE FOR FFT
SAMPLING_FREQUENCY = 192000 # SAMPLING FREQUENCY            Hz
TARGET_FREQUENCY = 1000     # FREQUENCY TO ACT UPON         Hz
FREQUENCY_RANGE = 300       # RANGE OFF TARGET TO MONITOR   Hz
SPEED = 343                 # SPEED OF SOUND IN AIR         m/s
MIC_INDEX = -1              # MIC PORT INDEX
LIN_INDEX = -2              # LINE IN PORT INDEX

# USEFUL CONSTANTS
FREQUENCY_PER_INDEX = SAMPLING_FREQUENCY / float(BUFFERSIZE)
TIME_PER_INDEX = 1 / SAMPLING_FREQUENCY
TARGET_INDEX = int(round(TARGET_FREQUENCY / FREQUENCY_PER_INDEX))
RANGE = int(round(FREQUENCY_RANGE / FREQUENCY_PER_INDEX))
RFFT_LENGTH = BUFFERSIZE / 2 + 1
LABELS = ['I', 'II', 'III', 'IV']

# SET UP CURSES
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
    __slots__ = ['label', 'time', 'freq', 'magn']

    def __init__(self, quadrant):
        ''' Initializes mic object '''
        self.reset(quadrant)

    def reset(self, quadrant):
        ''' Resets mic object '''
        self.label = LABELS[quadrant]
        self.time = np.zeros(BUFFERSIZE, np.float)
        self.freq = np.zeros(RFFT_LENGTH, np.float)
        self.magn = np.zeros(RFFT_LENGTH, np.float)

    def compute_fft(self):
        ''' FFTs time domain '''
        self.freq = np.fft.rfft(self.time)

    def compute_magnitude(self):
        ''' Computes magnitude '''
        np.seterr(divide='ignore') 
        self.magn = 20*np.log10(np.abs(self.freq))


# CREATE MIC OBJECTS
mics = [mic(i) for i in range(NUMBER_OF_MICS)]


# SET UP ERROR HANDLER TO FILTER OUT PYAUDIO MESSAGES
def py_error_handler(filename, line, function, err, fmt):
    pass

ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so')
asound.snd_lib_error_set_handler(c_error_handler)


# SET UP SOUND INPUT
try:
    audio = pyaudio.PyAudio()
    os.system('clear')
    mic = audio.open(format=pyaudio.paFloat32, channels=2,
                     rate=SAMPLING_FREQUENCY, input=True,
                     input_device_index = MIC_INDEX, frames_per_buffer=BUFFERSIZE)
    lin = audio.open(format=pyaudio.paFloat32, channels=2,
                     rate=SAMPLING_FREQUENCY, input=True,
                     input_device_index = LIN_INDEX, frames_per_buffer=BUFFERSIZE)
except Exception as e:
    screen.addstr('\n %s \n\n' % e, curses.color_pair(1))
    screen.refresh()
    time.sleep(1)
    curses.endwin()
    print e
    exit(1)


def read():
    ''' Reads time domain from microphones and updates objects '''
    data_mic = np.fromstring(mic.read(BUFFERSIZE), dtype=np.float32)
    data_lin = np.fromstring(lin.read(BUFFERSIZE), dtype=np.float32)

    mics[0].time = data_mic[::2]
    mics[1].time = data_mic[1::2]
    mics[2].time = data_lin[::2]
    mics[3].time = data_lin[1::2]


def process():
    ''' FFTs the time domain and computes its magnitude and phase '''
    for i in range(NUMBER_OF_MICS):
        mics[i].compute_fft()
        mics[i].compute_magnitude()


# LOOK AT RELEVANT FREQUENCY
def analyze():
    ''' Monitors target frequency and prints table '''
    target = 'TARGET\t  %4d Hz\n' % (TARGET_FREQUENCY)
    screen.addstr(0, 0, target)

    header = '\n%s\t%s\t%s\t\n\n' % (' MIC', 'FREQUENCY', 'MAGNITUDE')
    screen.addstr(header, curses.color_pair(3))

    for i in range(NUMBER_OF_MICS):
        for j in range(TARGET_INDEX - RANGE, TARGET_INDEX + RANGE + 1):
            value = '%s\t%5d Hz\t%+4.2f\tdB\n' % \
                    (' ' + mics[i].label,
                     j * FREQUENCY_PER_INDEX,
                     mics[i].magn[j])
            screen.addstr(value)

        screen.addstr('\n')


def maximize():
    ''' Finds max frequency and prints table '''
    screen.addstr(' MAX\t\t\t\t\t\n\n', curses.color_pair(3))
    for i in range(NUMBER_OF_MICS):
        max = np.argmax(mics[i].magn)

        state = 0
        if max >= TARGET_INDEX - RANGE and max <= TARGET_INDEX + RANGE:
            state = 2

        value = '%s\t%5d Hz\t%+4.2f\tdB\n' % \
                (' ' + mics[i].label,
                 max * FREQUENCY_PER_INDEX,
                 mics[i].magn[max])
        screen.addstr(value, curses.color_pair(state))


def close():
    ''' Closes audio streams '''
    audio.close(mic)
    audio.close(lin)


if __name__ == '__main__':
    # MAIN
    while True:
        try:
            read()
            process()
            analyze()
            maximize()
            screen.refresh()

        except:
            break

    # QUIT PEACEFULLY
    try:
        close()
        screen.addstr('\n GOODBYE \n\n', curses.color_pair(1))
        screen.refresh()
        time.sleep(1)

    finally:
        curses.endwin()
        exit(0)
