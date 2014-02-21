import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import dead_reck

#TODO: Finish everything in this file.

# Callbacks
def cvCallback(poseStamped):
    pos = poseStamped.pose.position

    estimator.updateCV([pos.x, pos.y, pos.z])

def imuCallback(poseStamped):
    orientation = poseStampe.pose.orientation

    estimator.updateOrientation([orientation.w, orientation.x, orientation.y, orientation.z])

def velCallback(velVector):
    estimator.updateVelocity([velVector.x, velVector.y, velVector.z])

def depthCallback(depth):
    estimator.updateVelocity(depth)

# Init the ros node, subscribers and publishers
# And run the node
def init():
    global estimator
    estimator = dead_reck.dead_reck()
    rospy.init_node('state_estimation')

    # Subscribe to different inputing topics
    rospy.Subscriber('cv_topic', PoseStamped, cvCallback)
    rospy.Subscriber('imu_topic', PoseStamped, imuCallback)
    rospy.Subscriber('vel_topic', Vector3, velCallback)
    rospy.Subscriber('depth_topic', Float64, depthCallback)

    # Publish the filtered data to a topic
    pub = rospy.publisher('state', PoseStamped)
    r = rospy.Rate(100) #100Hz

    while not rospy.is_shutdown():
        pub.publish(estimator.getState())
        r.sleep()
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass