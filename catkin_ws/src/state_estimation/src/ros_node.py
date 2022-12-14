import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float64
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from computer_vision.msg import VisibleObjectData
from state_estimation.msg import AUVState
import dead_reck
import math
import time

POOL_DEPTH = 6
SEE_OBJ = False
COUNTER = time.time()
THRESHOLD = 5   # IN SECONDS


def publish():
    global SEE_OBJ, COUNTER
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

    #TODO: velocity, pitch, yaw

    pub.publish(state_msg)
    see_obj_pub.publish(SEE_OBJ)

    if time.time() - COUNTER > THRESHOLD:
        SEE_OBJ = False

# Callbacks
def cvCallback(visObject):
    global SEE_OBJ, COUNTER
    hasTarget = (visObject.object_type != VisibleObjectData.CANNOT_DETERMINE_OBJECT)
    SEE_OBJ = hasTarget
    COUNTER = time.time()
    estimator.updateCV(hasTarget, visObject.object_type,
                       visObject.x_distance, visObject.y_distance, visObject.z_distance,
                       visObject.yaw_angle, visObject.pitch_angle)


def imuCallback(poseStamped):
    q = poseStamped.pose.orientation
    Q = [q.w, q.x, q.y, q.z]

    #print "Corrected {}".format(Q)
    #print "%3f %3f %3f"%(Q[1],Q[2],Q[3])

    sin = math.sqrt(Q[1]*Q[1] + Q[2]*Q[2] + Q[3]*Q[3])
    if sin != 0 and Q[3] != 0:
        sin *= Q[3]/math.fabs(Q[3])
    cos = Q[0]
    angle = 2*math.atan2(sin,cos)
    estimator.updateOrientation(angle)
    publish()


def depthCallback(depth):
    estimator.updateDepth(depth.data)
    down_distance_pub.publish(POOL_DEPTH - depth.data)

# Init the ros node, subscribers and publishers
# And run the node
def init():
    global estimator, pub, down_distance_pub, see_obj_pub

    # NODE
    rospy.init_node('state_estimation')
    estimator = dead_reck.dead_reck(rospy)

    # See object
    see_obj_pub = rospy.Publisher('state_estimation/see_object', Bool)
    # Publish the filtered data to a topic
    pub = rospy.Publisher('state_estimation/state_estimate', AUVState)
    down_distance_pub = rospy.Publisher('state_estimation/down_distance', Float64)

    # Subscribe to different inputing topics
    rospy.Subscriber('front_cv/data', VisibleObjectData, cvCallback)
    rospy.Subscriber('down_cv/data', VisibleObjectData, cvCallback)
    rospy.Subscriber('state_estimation/pose', PoseStamped, imuCallback)
    rospy.Subscriber('state_estimation/filteredDepth', Float64, depthCallback)

    # SET INITIAL IMU RPY PARAMS
    rospy.set_param('/IMU/initial/roll',0)
    rospy.set_param('/IMU/initial/pitch',0)
    rospy.set_param('/IMU/initial/yaw',0)

    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass