#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def callback(data):
    x = data.orientation.x
    y = data.orientation.y
    z = data.orientation.z
    w = data.orientation.w

    rospy.loginfo(rospy.get_name() + ": I heard: x, y, z, w: %f, %f, %f, %f", x, y, z, w)


def listener():
    rospy.init_node('poseSubscriber', anonymous=True)
    rospy.Subscriber("pose", Pose, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
