#!/usr/bin/env python

import serial
import rospy
from std_msgs.msg import String

def talker(str):
    pub = rospy.Publisher('times', String)
    rospy.init_node('hydrophones')
    rospy.loginfo(str)
    pub.publish(String(str))

ser = serial.Serial('/dev/teensy')
ser.open()

print('connecting to teensy...')

while not ser.isOpen():
    pass

print 'connected!'

while True:
    line = ser.readline().rstrip()
    if line is '.':
        continue
    if __name__ == '__main__':
        try:
            talker(line)
        except rospy.ROSInterruptException:
            pass