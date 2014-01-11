#!/usr/bin/env python

# IMPORTS
import time
import serial
import rospy
from std_msgs.msg import String

# VARIABLES
NUMBER_OF_MICS = 4
mics = [[] for x in xrange(NUMBER_OF_MICS)]

# ROS TOPIC
def publish(str):
    pub = rospy.Publisher('times', String)
    rospy.init_node('hydrophones')
    rospy.loginfo(str)
    pub.publish(String(str))

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
                    publish(line)
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