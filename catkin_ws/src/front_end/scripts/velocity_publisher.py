#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from planner.msg import MotorControl

def velocity_publisher(x_velo, y_velo, z_depth, pitch_vel, yaw_vel, ros_topic):

    #create the publisher for the cmd_vel topic
    vel_pub = rospy.Publisher(ros_topic, MotorControl)

    msg = MotorControl()
    # define the twist message from the joystick input
    msg.XSpeed= x_velo
    msg.YSpeed = y_velo
    msg.ZPos = z_depth

    msg.PitchSpeed = pitch_vel
    msg.YawSpeed = yaw_vel

    #rospy.loginfo(msg)

    vel_pub.publish(msg)

    #return str(msg)
