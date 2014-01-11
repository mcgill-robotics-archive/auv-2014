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

while True:
    line = ser.readline().rstrip()
    if line is '.':
        continue
    if __name__ == '__main__':
        try:
            publish(line)
        except rospy.ROSInterruptException:
            pass