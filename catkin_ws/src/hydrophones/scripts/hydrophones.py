#!/usr/bin/env python

# IMPORTS
import time
import serial
import rospy
import math
import numpy as np
import pyaudio
from std_msgs.msg import String

# VARIABLES
NUMBER_OF_MICS = 4  # SELF-EXPLANATORY
DISTANCE = 40       # DISTANCE BETWEEN HYDROPHONES
SPEED = 4000        # SPEED OF SOUND UNDER WATER

mics = [[] for x in xrange(NUMBER_OF_MICS)]
angles = [[] for x in xrange(2)]

FORMAT = pyaudio.paFloat32
SAMPLEFREQ = 96000
FRAMESIZE = 1024
p = pyaudio.PyAudio()

# ROS TOPIC
def publish():
    pub = rospy.Publisher('angles', String)
    rospy.init_node('hydrophones')
    rospy.loginfo(angles[0][0])
    pub.publish(String(angles[0][0]))

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
        mics[0].append(int(str[1:]))
    elif str[0] is 'X':
        mics[1].append(int(str[1:]))
    elif str[0] is 'Y':
        mics[2].append(int(str[1:]))
    elif str[0] is 'Z':
        mics[3].append(int(str[1:]))

# TRIANGULATE
def triangulate(index):
    # TIME DIFFERENCES
    dtx = mics[1][index] - mics[0][index]
    dty = mics[2][index] - mics[0][index]
    dtz = mics[3][index] - mics[0][index]

    # DISTANCE DIFFERENCES (ds = v * dt)
    sx = dtx * SPEED
    sy = dty * SPEED
    sz = dtz * SPEED

    # CALCULATE YAW
    # NOTE: the following was generated and optimized using Maple 
    t1 = DISTANCE * DISTANCE
    t3 = t1 * sy
    t5 = sy * sy
    t7 = sx * sx
    t8 = t7 * t5
    t9 = sy * t5
    t11 = t1 * t1
    t16 = t7 * t7
    t21 = t5 * t5
    t27 = sx * t7
    t35 = sx * t16
    t39 = t7 * t16
    t44 = 2 * t7 * t1 * t11 - 3 * t16 * t11 - 3 * t8 * t11 + t21 * t7 * t1 + 4 * t16 * t5 * t1 - 2 * t27 * t9 * t1 + 2 * t27 * sy * t11 - t16 * t21 - 2 * t35 * t1 * sy + t39 * t1 + 2 * t35 * t9 - t39 * t5
    t45 = math.sqrt(t44)
    t46 = t3 * sx - t1 * t5 +  t8 - t9 * sx + t11 - t1 * t7 + t45
    t48 = t1 - t5 - t7

    yaw = math.atan((t1 * sx + sy * t46 / t48 - t3 + t7 * sy - sx * t5) / sx / t46 * t48)

    # CALCULATE PITCH
    # NOTE: the following was generated and optimized using Maple 
    t3 = t1 * sz
    t5 = sz * sz
    t8 = t7 * t5
    t9 = sz * t5
    t21 = t5 * t5
    t44 = 2 * t7 * t1 * t11 - 3 * t16 * t11 - 3 * t8 * t11 + t21 * t7 * t1 + 4 * t16 * t5 * t1 - 2 * t27 * t9 * t1 + 2 * t27 * sz * t11 - t16 * t21 - 2 * t35 * t1 * sz + t39 * t1 + 2 * t35 * t9 - t39 * t5
    t45 = math.sqrt(t44)
    t46 = t3 * sx - t1 * t5 +  t8 - t9 * sx + t11 - t1 * t7 + t45
    t48 = t1 - t5 - t7

    pitch = math.atan((t1 * sx + sz * t46 / t48 - t3 + t7 * sz - sx * t5) / sx / t46 * t48)

    angles[0].append(yaw)
    angles[1].append(pitch)

    print yaw
    print pitch

# AVERAGE VALUES
def average():
    yawSum = 0
    pitchSum = 0
    length = len(angles[0])

    if length > 0:
        for i in xrange(length):
            yawSum += angles[0][i]
            pitchSum += angles[1][i]

        angles[0][0] = yawSum / length
        angles[1][0] = pitchSum / length

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
                        if len(mics[0]) > 0 and len(mics[0]) == len(mics[1]) and len(mics[1]) == len(mics[2]) and len(mics[2]) == len(mics[3]):
                            for i in xrange(len(mics[0])):
                                try:
                                    triangulate(i)
                                    average()
                                    publish()
                                except:
                                    print 'degenerate triangle'
                        elif len(mics[0]) > 0 and len(mics[1]) > 0 and len(mics[2]) > 0 and len(mics[3]) > 0:
                            try:
                                triangulate(0)
                                publish()
                            except:
                                print 'degenerate triangle'

                        else:
                            print 'pinger could not be found'

                        # RESET DATA
                        for i in xrange(NUMBER_OF_MICS):
                            del mics[i][:]
                        for i in xrange(2):
                            del angles[i][:]
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