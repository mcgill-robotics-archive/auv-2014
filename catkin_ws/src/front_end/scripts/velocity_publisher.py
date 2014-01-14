#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from planner.msg import MotorControl

def velocity_publisher(x_vel, y_vel, z_depth, pitch_vel, yaw_vel, ros_topic):

    #create the publisher for the cmd_vel topic
    vel_pub = rospy.Publisher(ros_topic, MotorControl)

    msg = MotorControl()
    # define the twist message from the joystick input
    msg.XSpeed= x_vel
    msg.YSpeed = y_vel
    msg.ZPos = z_depth

    msg.PitchSpeed = pitch_vel
    msg.YawSpeed = yaw_vel

    #rospy.loginfo(msg)

    vel_pub.publish(msg)

    #return str(msg)
