#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

import rospy
from planner.msg import setPoints

def velocity_publisher(x_vel, y_vel, z_depth, pitch_vel, yaw_vel, ros_topic):

    #create the publisher for the cmd_vel topic
    vel_pub = rospy.Publisher(ros_topic, setPoints)

    msg = setPoints()
    # define the twist message from the joystick input
    
    msg.XSpeed.data = x_vel
    msg.YSpeed.data = y_vel
    msg.Depth.data = z_depth

    msg.Pitch.data = pitch_vel
    msg.YawSpeed.data = yaw_vel


    msg.XPos.isActive=0
    msg.YPos.isActive=0
    msg.Depth.isActive=1
    msg.Yaw.isActive=0
    msg.Pitch.isActive=1
    msg.XSpeed.isActive=1
    msg.YSpeed.isActive=1
    msg.YawSpeed.isActive=1

    vel_pub.publish(msg)

    #return str(msg)
