#!/usr/bin/env python

# IMPORTS
import time
import serial
from numpy import linalg, fft
# import sys
# import rospy
# from geometry_msgs.msg import PointStamped
# from threading import Thread
from math import sqrt, sin, cos, pi

# VARIABLES
NUMBER_OF_MICS = 4  # SELF-EXPLANATORY
DIMENSIONS = 3      # DIMENSIONS OF SOLUTION
DISTANCE_X = 0.565  # DISTANCE M1 <---> M3
DISTANCE_Y = 0.349  # DISTANCE M2 <---> M3
SPEED = 343         # SPEED IN AIR
MAX_DELAY = 10000   # MAXIMUM DELAY BEFORE RESET (TICKS)
THRESHOLD = 10      # VOLUME THRESHOLD (OUT OF 128) - EXPERIMENTAL VALUE
TICK_TIME = 15      # TIME PER LOOP CYCLE (MICROSECONDS) - EXPERIMENTAL VALUE
TEENSY_PATH = "/dev/tty.usbmodem10131"  # 'dev/teensy'


# MIC OBJECT
class mic(object):
    __slots__ = ['value', 'peak', 'time', 'done']

    def __init__(self):
        self.reset()

    def reset(self):
        self.value = 0
        self.peak = 0
        self.time = 0
        self.done = False


# CREATE MIC OBJECTS
mics = [mic() for x in xrange(NUMBER_OF_MICS)]


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

    print 'connected!'

    return ser


# PARSE
def parse(str):
    if str[0] is '0':
        mics[0].value = abs(long(str[1:]) - 128)
    elif str[0] is '1':
        mics[1].value = abs(long(str[1:]) - 128)
    elif str[0] is '2':
        mics[2].value = abs(long(str[1:]) - 128)
    elif str[0] is '3':
        mics[3].value = abs(long(str[1:]) - 128)
        return True

    return False


# TRIANGULATE
def triangulate(dtx, dty):
    sx = dtx*SPEED  # Distance M1 <---> M2
    sy = dty*SPEED  # Distance M1 <---> M3

    phi = pi / 2  # Angle between M1 and M2 in rads

    # Initial Guess
    r1 = 10
    theta = 90

    # Solution vector
    Y = [r1, theta]

    # Position Vector
    R = [Y[0], sx, sy, DISTANCE_X, DISTANCE_Y]
    F = [0, 0]
    J = [[0, 0], [0, 0]]

    # Tolerance
    t = 1e-9
    e = 100

    # Iteration count
    i = 0

    # As long as the error is higher than the tolerance
    while e > t:
        R = [Y[0], sx, sy, DISTANCE_X, DISTANCE_Y]
        theta = Y[1]

        # Calculate the function
        F = [(R[0] + R[2])**2 - (R[0] + R[1])**2 + R[3]**2 - 2*(R[0] + R[2]) * R[3] * cos(theta),
             (R[0] + R[2])**2 + R[4]**2 - R[0]**2 - 2 * (R[0]+R[2]) * R[4] * cos(theta-phi)]

        J[0][0] = 2 * R[2] - 2 * R[1] - 2 * R[3] * cos(theta)
        J[0][1] = 2 * (R[0] + R[2]) * R[3] * sin(theta)
        J[1][0] = 2 * R[2] - 2 * R[4] * cos(theta - phi)
        J[1][1] = 2 * (R[0] + R[2]) * R[4] * sin(theta - phi)

        delta_x = -linalg.solve(J, F)

        Y += delta_x

        e = sqrt(delta_x[0]**2 + delta_x[1]**2)

        if i > 1000:
            print "WARNING: Did not converge within ", i, " iterations with e = ", e
            e = 0

        i += 1

    x = Y[0] * cos(Y[1])
    y = Y[0] * sin(Y[1])

    coordinates = [x, y]

    return coordinates


# COMPUTE
def everything():
    # INCREMENT TIMER AND CHECK FOR PEAKS
    for i in xrange(NUMBER_OF_MICS):
        if mics[i].value > mics[i].peak:
            # found peak: reset time
            mics[i].peak = mics[i].value
            mics[i].time = 0
        else:
            mics[i].time += 1
            if mics[i].time > MAX_DELAY:
                # max_delay has passed
                mics[i].done = True

    # CHECK IF ALL ARE DONE
    for i in xrange(NUMBER_OF_MICS):
        if not mics[i].done:
            # not done: keep listening
            return

    # CHECK IF ALL ARE ABOVE THRESHOLD
    all = True
    for i in xrange(NUMBER_OF_MICS):
        if mics[i].peak < THRESHOLD:
            all = False
            print mics[i].peak
            break

    # PERFORM CALCULATIONS
    if all:
        dtx = (mics[2].time - mics[0].time) * TICK_TIME
        dty = (mics[1].time - mics[0].time) * TICK_TIME

        print triangulate(dtx, dty)

    # RESET
    for i in xrange(NUMBER_OF_MICS):
        mics[i].reset()


# MAIN
try:
    # CONNECT
    ser = connect()

    # FRUIT LOOPS
    while True:
        try:
            # READ SERIAL DATA AND PUBLISH TOPIC
            line = ser.readline().rstrip()

            # if __name__ == '__main__':
            #     try:
            #         if parse(line):
            #             everything()
            #     except rospy.ROSInterruptException:
            #         pass

            if parse(line):
                everything()
        except serial.serialutil.SerialException:
            # PEACE OUT IF CONNECTION DROPS
            print 'connection dropped'
            time.sleep(1)
            print 'exiting...'
            exit(1)

except KeyboardInterrupt:
    # CTRL-C FRIENDLY
    print ''
    print 'goodbye!'
    ser.close()
    time.sleep(1)
    exit(0)
