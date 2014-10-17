#!/usr/bin/env python

"""Reinitialize IMU orientation."""

# IMPORTS
import rospy
import tf
import time
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# ROSPY AND TF
rospy.init_node('initializer')
listener = tf.TransformListener()


class RPY(object):

    """Roll, Pitch and Yaw object."""

    def __init__(self, roll, pitch, yaw):
        """Construct RPY object.

        :param roll: roll
        :type roll: float
        :param pitch: pitch
        :type pitch: float
        :param yaw: yaw
        :type yaw: float
        """
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)


# CUMMULATIVE RPY DATA
lay_of_the_land = []


def imu_callback(pose):
    """Callback on state estimation pose.

    :param pose: state estimation pose
    :type pose: PoseStamped ROS message
    """
    # DISREGARD MESSAGE, LOOK AT TF INSTEAD
    try:
        (trans, rot) = listener.lookupTransform(
            # FROM
            '/sensors/raw/IMU_global_reference',
            # TO
            '/robot/rotation_center',
            # NOW
            rospy.Time(0)
        )
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException):
        pass

    # GET ROLL, PITCH, YAW FROM QUATERNION
    (roll, pitch, yaw) = euler_from_quaternion(rot)

    # APPEND DATA
    lay_of_the_land.append(
        RPY(roll, pitch, yaw)
    )


def timer(countdown):
    """Hold thread for specified time and print progress.

    :param countdown: number of seconds to wait
    :type countdown: int
    """
    start = time.time()
    time_elapsed = 0
    while time_elapsed < countdown:
        time_elapsed = time.time() - start
        progress = time_elapsed / countdown * 100
        print '\r{progress:>3.0f}%'.format(progress=progress),
    print '\rdone'


def reinitialize_imu(countdown=5):
    """Reinitialize IMU heading parameters.

    :param countdown: number of seconds to gather data format_progress
    :type countdown: int
    """
    # RESET CUMMULATIVE DATA
    del lay_of_the_land[:]

    # SUBSCRIBE FOR SPECIFIED AMOUNT OF TIME TO GATHER DATA
    print 'initializing...'
    imu = rospy.Subscriber('state_estimation/pose', PoseStamped, imu_callback)
    timer(countdown)
    imu.unregister()

    # COMPUTE MEAN
    mean = RPY(
        np.median([orientation.roll for orientation in lay_of_the_land]),
        np.median([orientation.pitch for orientation in lay_of_the_land]),
        np.median([orientation.yaw for orientation in lay_of_the_land])
    )

    # SET PARAMETERS
    rospy.set_param('/IMU/initial/roll', mean.roll)
    rospy.set_param('/IMU/initial/pitch', mean.pitch)
    rospy.set_param('/IMU/initial/yaw', mean.yaw)
    rospy.set_param('/IMU/reinitialized', True)

    # LOG
    rospy.logwarn('FILTERED %d DATA POINTS', len(lay_of_the_land))
    rospy.logwarn('INITIAL ROLL: %f', mean.roll)
    rospy.logwarn('INITIAL PITCH:%f', mean.pitch)
    rospy.logwarn('INITIAL YAW:  %f', mean.yaw)


if __name__ == '__main__':
    reinitialize_imu()
