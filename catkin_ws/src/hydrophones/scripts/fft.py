#!/usr/bin/env python

# IMPORTS
import time
import serial
from numpy import fft, absolute, around
import numpy as np
from math import sqrt, sin, cos, pi

# VARIABLES
NUMBER_OF_MICS = 4      # SELF-EXPLANATORY
BLOCK_SIZE = 250        # FFT BLOCK SIZE
DISTANCE = 0.565        # DISTANCE BETWEEN MICS
TEENSY_PATH = "/dev/tty.usbmodem27521"  # 'dev/teensy'

# CREATE MIC OBJECTS
mics = [[] for x in xrange(NUMBER_OF_MICS)]
freq = [[] for x in xrange(NUMBER_OF_MICS)]


# ESTABLISH CONNECTION
def connect():
    print 'connecting to teensy...'

    # KEEP TRYING UNTIL WORKS
    done = False
    while not done:
        try:
            ser = serial.Serial(TEENSY_PATH)
            done = True
        except serial.serialutil.SerialException:
            print 'connection failed'
            time.sleep(1)
        except OSError:
            print 'teensy not connected'
            print 'goodbye!'
            time.sleep(1)
            exit(0)            

    print 'connected!'

    return ser


# PARSE
def parse(str):
    if str[0] is '0':
        mics[0].append(float(str[1:]))
    elif str[0] is '1':
        mics[1].append(float(str[1:]))
    elif str[0] is '2':
        mics[2].append(float(str[1:]))
    elif str[0] is '3':
        mics[3].append(float(str[1:]))
        return True

    return False


# FOURIER TRANSFORM
def fourier():
    freq[0] = fft.rfft(mics[0])
    freq[1] = fft.rfft(mics[1])
    freq[2] = fft.rfft(mics[2])
    freq[3] = fft.rfft(mics[3])

    test = (absolute(freq[0])).astype(int)

    print '\t'.join([str(x) for x in test[95:106]])

# MAIN
try:
    # CONNECT
    ser = connect()
    counter = 0

    # FRUIT LOOPS
    while True:
        try:
            # READ SERIAL DATA AND PARSE
            line = ser.readline().rstrip()
            if parse(line):
                counter += 1

            # WAIT TIL ENOUGH INFORMATION
            if counter == BLOCK_SIZE:
                # FOURIER TRANSFORM
                fourier()

                # RESET COUNTER AND DATA
                counter = 0
                for i in xrange(NUMBER_OF_MICS):
                    mics[i] = []
        
        # PEACE OUT IF CONNECTION DROPS
        except serial.serialutil.SerialException:
            print 'connection dropped'
            time.sleep(1)
            print 'exiting...'
            exit(1)

# CTRL-C FRIENDLY
except KeyboardInterrupt:
    print ''
    print 'goodbye!'
    ser.close()
    time.sleep(1)
    exit(0)
