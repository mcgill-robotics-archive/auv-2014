#!/usr/bin/env python
#__author__ = 'david lavoie-boutin'

##
#@package velocity_publisher
#contains a method publishing a set of data to the setPoints topic in ros
#@author David Lavoie-Boutin

import rospy
from planner.msg import setPoints
from VARIABLES import vel_vars

##
#publishes the passed data to ros
#@param x_vel velocity in the x axis float 64-bits
#@param y_vel velocity in y axis float 64-bits
#@param z_depth desired depth, positive pointing down float 64-bits
#@param yaw_vel angular velocity for yaw float 64-bits
#@param ros_topic the topic name to which you publish float 64-bits
#@param set_null if set to 0, sends all controls NOT active
def velocity_publisher(y_vel, ros_topic, set_null, depth_speed_control):
    vel_pub = rospy.Publisher(ros_topic, setPoints)

    msg = setPoints()

    if set_null != 0:
        #create the publisher for the cmd_vel topic

        # define the twist message from the joystick input
        
        msg.XSpeed.data = vel_vars.x_velocity
        msg.YSpeed.data = y_vel

        if depth_speed_control:
            msg.DepthSpeed.data = vel_vars.z_position
            msg.DepthSpeed.isActive = 1
            msg.Depth.isActive = 0
        else:
            msg.Depth.data = vel_vars.z_position
            msg.Depth.isActive = 1
            msg.DepthSpeed.isActive = 0

        msg.YawSpeed.data = vel_vars.yaw_velocity

        msg.XPos.isActive = 0
        msg.YPos.isActive = 0
        msg.Yaw.isActive = 0
        msg.Pitch.isActive = 0
        msg.XSpeed.isActive = 1
        msg.YSpeed.isActive = 1
        msg.YawSpeed.isActive = 1
        msg.Frame=rospy.get_param('/tf_frame_fe')
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
        msg.DepthSpeed.isActive = 0
        vel_pub.publish(msg)

