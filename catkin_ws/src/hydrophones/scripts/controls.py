#!/usr/bin/env python
import rospy
import tf

from tf.transformations import euler_from_quaternion
from hydrophones.msg import *
from planner.msg import *

# NODE AND PUBLISHERS
rospy.init_node('hydrophone_controls')
pub_setPoints = rospy.Publisher('/planner/setPoints', setPoints)
start = rospy.Service('start_hydrophone_controls', StartHydrophoneTask, start_controller)

# Constants
openLoopSpeed = -1.0

# VARIABLES
start_hydrophones = False
kp = 10
ki = 0
integralError = 0

def start_controller(req):
    start_hydrophones = req.start
    return 0

def control(data):
    if(data.target):
        if(start_hydrophones):
            error = data.tdoa_1
            integralError = integralError + error

            try:
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
    
if __name__ == '__main__':
    try:
        rospy.Subscriber('/hydrophones/tdoa',tdoa,control)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
