#!/usr/bin/env python

# IMPORTS
import time
import serial
import rospy
from math import sqrt, pow
import numpy as np
import pyaudio
from geometry_msgs.msg import PointStamped
import sys

# VARIABLES
NUMBER_OF_MICS = 4  # SELF-EXPLANATORY
DISTANCE = 0.25     # DISTANCE BETWEEN HYDROPHONES IN CENTIMETERS
SPEED = 0.000330    # SPEED OF SOUND UNDER WATER IN METERS PER MICROSECOND
DIMENSIONS = 3      # DIMENSIONS OF SOLUTION

mics = [[] for x in xrange(NUMBER_OF_MICS)]
pos = [[] for x in xrange(DIMENSIONS)]
count = 0

pnt = rospy.Publisher('pos', PointStamped)
rospy.init_node('hydrophones')

# ROS TOPIC
def publish():
    global count

    pointStamp = PointStamped()
    pointStamp.header.seq = count
    pointStamp.header.stamp = rospy.rostime.Time.now()
    pointStamp.header.frame_id = 'map'
    pointStamp.point.x = pos[0][0]
    pointStamp.point.y = pos[1][0]
    pointStamp.point.z = pos[2][0]

    count += 1

    rospy.loginfo(pointStamp)
    pnt.publish(pointStamp)

# ESTABLISH CONNECTION
def connect():
    print 'connecting to teensy...'

    # KEEP TRYING UNTIL WORKS
    done = False
    while not done:
        try:
            ser = serial.Serial('/dev/teensy')
            ser.open()
            done = True
        except serial.serialutil.SerialException:
            print 'connection failed'
            time.sleep(1)

    print 'connected!'

    return ser

# PARSE
def parse(str):
    if str[0] is 'O':
        mics[0].append(long(str[1:]))
    elif str[0] is 'X':
        mics[1].append(long(str[1:]) + 1004)
    elif str[0] is 'Y':
        mics[2].append(long(str[1:]) + 70)
    elif str[0] is 'Z':
        mics[3].append(long(str[1:]) + 980)

# MAGNITUDE OF VECTOR
def magnitude(x, y, z):
    return sqrt(x * x + y * y )       

# TRIANGULATE
def triangulate(index):
    # TIME DIFFERENCES
    dtx = mics[0][index] - mics[1][index]
    dty = mics[0][index] - mics[2][index]
    dtz = mics[0][index] - mics[3][index]

    # DISTANCE DIFFERENCES (ds = v * dt)
    sx = dtx * SPEED
    sy = dty * SPEED
    sz = dtz * SPEED

    # FIRST SOLUTION
    x0 = (DISTANCE * DISTANCE * sy - sx * (DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) - DISTANCE * DISTANCE * sx + sx * sy * sy - sx * sx * sy) / DISTANCE / sy / 2
    y0 = -(DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) / DISTANCE / 2
    z0 = (DISTANCE * DISTANCE * sy - sz * (DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) - DISTANCE * DISTANCE * sz + sy * sy * sz - sy * sz * sz) / DISTANCE / sy / 2
    
    # SECOND SOLUTION
    x1 = (DISTANCE * DISTANCE * sy + sx * (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) - DISTANCE * DISTANCE * sx + sy * sy * sx - sy * sx * sx) / DISTANCE / sy / 2;
    y1 = (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) / DISTANCE / 2;
    z1 = (DISTANCE * DISTANCE * sy + sz * (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) - DISTANCE * DISTANCE * sz + sy * sy * sz - sy * sz * sz) / DISTANCE / sy / 2;

    # SELECT CORRECT SOLUTION
    minTime = min(dtx, min(dty, dtz))
    if dtx == minTime:
        if magnitude(x0 - DISTANCE, y0, z0) < magnitude(x1 - DISTANCE, y1, z1):
            x = x0
            y = y0
            z = z0
        else:
            x = x1
            y = y1
            z = z1
    elif dty == minTime:
        if magnitude(x0, y0 - DISTANCE, z0) < magnitude(x1, y1 - DISTANCE, z1):
            x = x0
            y = y0
            z = z0
        else:
            x = x1
            y = y1
            z = z1
    else:
        if magnitude(x0, y0, z0 - DISTANCE) < magnitude(x1, y1, z1 - DISTANCE):
            x = x0
            y = y0
            z = z0
        else:
            x = x1
            y = y1
            z = z1

    pos[0].append(x)
    pos[1].append(y)
    pos[2].append(z)

# AVERAGE VALUES
def average():
    xSum = 0
    ySum = 0
    zSum = 0
    length = len(pos[0])

    if length > 0:
        for i in xrange(length):
            xSum += pos[0][i]
            ySum += pos[1][i]
            zSum += pos[2][i]

        pos[0][0] = xSum / length
        pos[1][0] = ySum / length
        pos[2][0] = zSum / length

# MAIN
try:
    # CONNECT
    ser = connect()

    # FRUIT LOOPS
    while True:
        try:
            # READ SERIAL DATA AND PUBLISH TOPIC
            line = ser.readline().rstrip()

            if __name__ == '__main__':
                try:
                    if line is '.':
                        # CHECK IF NO FUCKUPS
                        #if len(mics[0]) > 0 and len(mics[0]) == len(mics[1]) and len(mics[1]) == len(mics[2]) and len(mics[2]) == len(mics[3]):
                            # for i in xrange(len(mics[0])):
                            #     #try:
                            #     triangulate(i)
                            #     average()
                            #     publish()
                                #except:
                                 #   print 'degenerate triangle'
                        #elif len(mics[0]) > 0 and len(mics[1]) > 0 and len(mics[2]) > 0 and len(mics[3]) > 0:
                        if True:
                            try:
                                triangulate(0)
                                publish()
                            except:
                                print 'degenerate triangle'

                        #else:
                            #print 'pinger could not be found'

                        # RESET DATA
                        for i in xrange(NUMBER_OF_MICS):
                            del mics[i][:]
                        for i in xrange(DIMENSIONS):
                            del pos[i][:]
                    else:
                        parse(line)
                except rospy.ROSInterruptException:
                    pass

        except serial.serialutil.SerialException:
            # PEACE OUT IF CONNECTION DROPS
            print 'connection dropped'
            time.sleep(1)
            print 'exiting...'
            exit(1)

except KeyboardInterrupt:
    # CTRL-C FRENDLY
    print ''
    print 'goodbye!'
    time.sleep(1)
    exit(0)