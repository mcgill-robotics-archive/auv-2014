#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from planner.msg import setPoints


def velocity_publisher(x_vel, y_vel, z_depth, pitch_vel, yaw_vel, ros_topic, set_null):
    vel_pub = rospy.Publisher(ros_topic, setPoints)

    msg = setPoints()

    if set_null != 0:
        #create the publisher for the cmd_vel topic

        # define the twist message from the joystick input
        
        msg.XSpeed.data = x_vel
        msg.YSpeed.data = y_vel
        msg.Depth.data = z_depth

        msg.Pitch.data = pitch_vel
        msg.YawSpeed.data = yaw_vel


        msg.XPos.isActive = 0
        msg.YPos.isActive = 0
        msg.Depth.isActive = 1
        msg.Yaw.isActive = 0
        msg.Pitch.isActive = 1
        msg.XSpeed.isActive = 1
        msg.YSpeed.isActive = 1
        msg.YawSpeed.isActive = 1

        vel_pub.publish(msg)
    else:
        # close down the channel for the planner to start
        
        msg.XSpeed.data = 0
        msg.YSpeed.data = 0
        msg.Depth.data = 0

        msg.Pitch.data = 0
        msg.YawSpeed.data = 0


        msg.XPos.isActive = 0
        msg.YPos.isActive = 0
        msg.Depth.isActive = 0
        msg.Yaw.isActive = 0
        msg.Pitch.isActive = 0
        msg.XSpeed.isActive = 0
        msg.YSpeed.isActive = 0
        msg.YawSpeed.isActive = 0

        vel_pub.publish(msg)

    #return str(msg)
