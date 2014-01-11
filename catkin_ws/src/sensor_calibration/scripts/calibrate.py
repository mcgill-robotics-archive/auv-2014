#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped

gyroX = []
gyroY = []
gyroZ = []
    

def gyroCallback(data):
    global gyroX, gyroY, gyroZ
    gyroX.append(data.vector.x)
    gyroY.append(data.vector.y)
    gyroZ.append(data.vector.z)

    

    xOffset = sum(gyroX)/len(gyroX)
    yOffset = sum(gyroY)/len(gyroY)
    zOffset = sum(gyroZ)/len(gyroZ)

    print("x offset: {}\ny offset: {}\nz offset: {}\n"\
        .format(xOffset,yOffset,zOffset))
    
    

def init():
    rospy.init_node('sensor_calibration')
    rospy.Subscriber('gyro_data', Vector3Stamped, gyroCallback)
    global pub
    pub = rospy.Publisher('gyro_offsets', String)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
