#!/usr/bin/env python

# IMPORTS
import time
import serial
import rospy
from std_msgs.msg import String

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
            exit(0)
except KeyboardInterrupt:
    print ''
    print 'goodbye!'
    time.sleep(1)
    exit(0)