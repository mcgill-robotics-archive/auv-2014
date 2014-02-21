import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import dead_reck

#TODO: Finish everything in this file.

def cvCallback(poseStamped):
    pos = poseStamped.pose.position;

    updateCV([pos.x, pos.y, pos.z], )

def init():
    global estimator
    estimator = dead_reck.dead_reck()
    rospy.init_node('state_estimation')
    rospy.subscriber('cv_topic', PoseStamped, cvCallback)
    rospy.subscriber('imu_topic', PoseStamped, imuCallback)
    rospy.subscriber('vel_topic', Vector3, velCallback)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass