#!/usr/bin/env python

import time
import serial
import rospy
from std_msgs.msg import String

def publish(str):
    pub = rospy.Publisher('times', String)
    rospy.init_node('hydrophones')
    rospy.loginfo(str)
    pub.publish(String(str))

def connect():
    print 'connecting to teensy...'

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

ser = connect()

while True:
    try:
        line = ser.readline().rstrip()
        if __name__ == '__main__':
            try:
                publish(line)
            except rospy.ROSInterruptException:
                pass
    except serial.serialutil.SerialException:
        print 'connection dropped'
        time.sleep(1)
        print 'exiting...'
        exit()