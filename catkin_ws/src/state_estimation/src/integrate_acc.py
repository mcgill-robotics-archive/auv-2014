#!/usr/bin/env python2.7
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
import numpy as np
import QuaternionUtils as QU

position = np.matrix([[0.0],[0.0],[0.0]])
velocity = np.matrix([[0.0],[0.0],[0.0]])
orientation = np.matrix([[1],[0],[0],[0]])
time = None

def poseCallback(pose):
    global orientation
    o = pose.pose.orientation
    orientation = np.matrix([[o.w],[-o.x],[-o.y],[-o.z]])
    pose.pose.position.x = float(position[0])
    pose.pose.position.y = float(position[1])
    pose.pose.position.z = float(position[2])
    pose.header.frame_id = 'base_link'
    pub.publish(pose)


def accCallback(acc):
    global time, position, velocity
    if time:
        dt = (rospy.Time.now() - time).to_sec()
        acc = np.matrix([[acc.x],[-acc.z],[acc.y]])
        g = QU.rotateVectorByQuaternion(np.matrix([[0],[0],[1.0]]), orientation)
        acc = 9.8*(acc - g)
        velocity += acc*dt
        position += velocity*dt+acc*dt*dt/2
    time = rospy.Time.now()

# Init the ros node, subscribers and publishers
# And run the node
def init():
    global estimator, pub

    rospy.init_node('integrate_acc')
    # Subscribe to different inputing topics
    rospy.Subscriber('/state_estimation/pose', PoseStamped, poseCallback)
    rospy.Subscriber('/state_estimation/acc', Vector3, accCallback)

    # Publish the filtered data to a topic
    pub = rospy.Publisher('integrated_acc', PoseStamped)
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
