#!/usr/bin/env python

# import serial
# import rospy
# from std_msgs.msg import String

# rospy.init_node('hydrophones')
# micO = rospy.Publisher('micO', String)
# micX = rospy.Publisher('micX', String)
# micY = rospy.Publisher('micY', String)
# micZ = rospy.Publisher('micZ', String)

# def micO(time):
#     rospy.loginfo(time)
#     micO.publish(String(time))

# def micX(time):
#     rospy.loginfo(time)
#     micX.publish(String(time))

# def micY(time):
#     rospy.loginfo(time)
#     micY.publish(String(time))

# def micZ(time):
#     rospy.loginfo(time)
#     micZ.publish(String(time))

# ser = serial.Serial('/dev/teensy')
# ser.open()

# print('connecting to teensy...')

# while not ser.isOpen():
#     pass

# print 'connected!'

# while True:
#     line = ser.readline().rstrip()
#     if line is '.':
#         continue
#     if __name__ == '__main__':
#         try:
#             if line[0] is 'O':
#                 micO(line)
#             elif line[0] is 'X':
#                 micX(line)
#             elif line[0] is 'Y':
#                 micY(line)
#             elif line[0] is 'Z':
#                 micZ(line)
#         except rospy.ROSInterruptException:
#             pass

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