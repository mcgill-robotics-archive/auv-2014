#!/usr/bin/env python
import rospy
import tf

from tf.transformations import euler_from_quaternion
from hydrophones.msg import *
from hydrophones.srv import *
from planner.msg import *

# Constants
openLoopSpeed = -2.0
kp = 10
ki = 0
freq = 2

# VARIABLES
start_hydrophones = False
integralError = 0
desiredYaw = 0

def start_controller(req):
    global start_hydrophones
    rospy.logwarn('This happened. %s', str(req.start))
    start_hydrophones = req.start
    return 0

# NODE AND PUBLISHERS
rospy.init_node('hydrophone_controls')
pub_setPoints = rospy.Publisher('/setPoints', setPoints)
start = rospy.Service('start_hydrophone_controls', StartHydrophoneTask, start_controller)

def update_control(data):
    global integralError
    global desiredYaw
    
    rospy.logwarn('This also really happened. %s:%s' , str(start_hydrophones), str(data.target))

    if(data.target):
        error = data.tdoa_1
        rospy.logwarn('This also really fucking happened. %s' , str(start_hydrophones))

        integralError = integralError + error

        try:
            listener = tf.TransformListener()
            (trans,rot) = listener.lookupTransform(
                # from
                '/sensors/IMU_global_reference',
                # to
                '/robot/rotation_center',
                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # GET ROLL, PITCH, YAW FROM QUATERNION
        (roll,pitch,yaw) = euler_from_quaternion(rot)

        desiredYaw = yaw + kp*error + ki*integralError

def control():
    rospy.logwarn('This also happened. %s' , str(start_hydrophones))
    if(start_hydrophones):

        desiredSetPoint = setPoints(ValueControl(0,0),
                                ValueControl(0,0),
                                ValueControl(1,desiredYaw),
                                ValueControl(0,0),
                                ValueControl(1,openLoopSpeed),
                                ValueControl(0,0),
                                ValueControl(0,0),
                                ValueControl(1,rospy.get_param('/closeLoopDepth')),
                                ValueControl(0,0),
                                '/sensors/IMU_global_reference')
        
        pub_setPoints.publish(desiredSetPoint)
    
if __name__ == '__main__':
    rate = rospy.Rate(freq)
    try:
        rospy.Subscriber('/hydrophones/tdoa',tdoa,update_control)
        while not rospy.is_shutdown():
            control()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
