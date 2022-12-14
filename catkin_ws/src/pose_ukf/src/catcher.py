#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Vector3,Vector3Stamped,Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy
import math


accVariance = (9.8/256)*(9.8/256)
orientationVariance = math.atan(accVariance/9.8)
gyroVariance = (0.38*math.pi/180)*(0.38*math.pi/180)

def accVectorToQuat(vector):
    z = numpy.array([0,0,1])
    a = numpy.array([vector.x,vector.y,vector.z])
    aunit = a / numpy.linalg.norm(a)
    
    cross = numpy.cross(z,a)
    sin = numpy.linalg.norm(cross)
    cos = a.dot(z)

    if sin==0:
        #This means acc is parallel to z
        if cos > 0:
            #acc points in z direction
            return [1,0,0,0]
        else:
            #acc points -z direction
            return [0,1,0,0]

    angle = math.atan2(sin,cos)
    halfsin = math.sin(angle)
    q = []
    q.append(math.cos(angle))
    q += (halfsin * cross / numpy.linalg.norm(cross)).tolist()
    return q
    
def listToVector3(l):
    vector3 = Vector3()
    vector3.x = l[0]
    vector3.y = l[1]
    vector3.z = l[2]
    return vector3

def listToQuaternion(l):
    q = Quaternion()
    q.w = l[0]
    q.x = l[1]
    q.y = l[2]
    q.z = l[3]
    return q
    
def getCovariance(variance):
    cov = [variance]
    cov += [0,0,0]
    cov.append(variance)
    cov += [0,0,0]
    cov.append(variance)
    return cov

def accCallback(data):
    # Use data.vector to get vector
    # 
    imu_msg = Imu()
    q = accVectorToQuat(data.vector)
    
    header = Header()
    header.stamp = data.header.stamp

    orientationCovariance = getCovariance(orientationVariance)
    orientationCovariance[8] = 1000000
	
    imu_msg.header = header
    imu_msg.orientation = listToQuaternion(q)
    imu_msg.orientation_covariance = orientationCovariance
    imu_msg.angular_velocity = omega
    imu_msg.angular_velocity_covariance = getCovariance(gyroVariance)
    #Remove minus from y and z to switch to NED coord system
    imu_msg.linear_acceleration.x = data.vector.y
    imu_msg.linear_acceleration.y = -(data.vector.x)
    imu_msg.linear_acceleration.z = -(data.vector.z)
    imu_msg.linear_acceleration_covariance = getCovariance(accVariance)
    pub.publish(imu_msg)

def gyroCallback(data):
    global omega
    #Remove minus from y and z to switch to NED coord system
    omega.x = -data.vector.y+0.137
    omega.y = -(-data.vector.x)
    omega.z = -(-data.vector.z)

def init():
    rospy.init_node('sensor_catcher')
    rospy.Subscriber('acc_data', Vector3Stamped, accCallback)
    rospy.Subscriber('gyro_data', Vector3Stamped, gyroCallback)
    global omega
    omega = listToVector3([0,0,0])
    global pub
    pub = rospy.Publisher('imu_data', Imu)
    rospy.spin()


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
