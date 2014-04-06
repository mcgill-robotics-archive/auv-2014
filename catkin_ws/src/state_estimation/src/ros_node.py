import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from computer_vision.msg import VisibleObjectData
from state_estimation.msg import AUVState
import dead_reck
import math


def publish():
    state = estimator.getState()

    state_msg = AUVState()
    vis_msg = VisibleObjectData()
    vis_msg.object_type = state['targetID']
    vis_msg.x_distance = state['x']
    vis_msg.y_distance = state['y']
    vis_msg.z_distance = state['z']
    vis_msg.yaw_angle = state['yawOfTarget']
    vis_msg.pitch_angle = state['pitchOfTarget']

    state_msg.visibleObjectData = vis_msg
    state_msg.hasTarget = state['hasTarget']
    state_msg.depth = state['depth']

    #Todo: velocity, pitch, yaw

    pub.publish(state_msg)

# Callbacks
def cvCallback(visObject):
    hasTarget = visObject.object_type != VisibleObjectData.CANNOT_DETERMINE_OBJECT
    #TODO: remove conversion to angle once CV publishes in radians
    estimator.updateCV(hasTarget, visObject.object_type,
                       visObject.x_distance, visObject.y_distance, visObject.z_distance,
                       visObject.yaw_angle*math.pi/180, visObject.pitch_angle*math.pi/180)


def imuCallback(poseStamped):
    q = poseStamped.pose.orientation
    Q = [q.w, q.x, q.y, q.z]


    #print "Corrected {}".format(Q)
    print "%3f %3f %3f"%(Q[1],Q[2],Q[3])

    sin = math.sqrt(Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3])
    cos = Q[0]
    angle = 2*math.atan2(sin,cos)

    estimator.updateOrientation(angle)
    publish()



def depthCallback(depth):
    estimator.updateDepth(depth.data)

# Init the ros node, subscribers and publishers
# And run the node
def init():
    global estimator, pub

    rospy.init_node('state_estimation')
    estimator = dead_reck.dead_reck(rospy)
    # Subscribe to different inputing topics
    rospy.Subscriber('front_cv/data', VisibleObjectData, cvCallback)
    rospy.Subscriber('down_cv/data', VisibleObjectData, cvCallback)
    rospy.Subscriber('state_estimation/pose', PoseStamped, imuCallback)
    rospy.Subscriber('state_estimation/depth', Float64, depthCallback)

    # Publish the filtered data to a topic
    pub = rospy.Publisher('state_estimation/state_estimate', AUVState)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass