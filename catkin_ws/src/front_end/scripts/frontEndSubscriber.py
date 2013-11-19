#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

def pose_callback(pose_data):
    x = pose_data.orientation.x
    y = pose_data.orientation.y
    z = pose_data.orientation.z
    w = pose_data.orientation.w

    rospy.loginfo(rospy.get_name() + ": orientation : x, y, z, w: %f, %f, %f, %f", x, y, z, w)

def depth_callback(depth_data):
    depth = depth_data.data;

    rospy.loginfo(rospy.get_name() + ": depth : %f", depth)

def pressure_callback(pressure_data):
    pressure = pressure_data.data;

    rospy.loginfo(rospy.get_name() + ": pressure : %f", pressure)

def listener():
    rospy.init_node('frontEndSubscriber', anonymous=True)
    rospy.Subscriber("pose", Pose, pose_callback)
    rospy.Subscriber("depth", Float32, depth_callback)
    rospy.Subscriber("pressure", Float32, pressure_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
