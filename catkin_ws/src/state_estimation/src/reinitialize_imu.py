#!/usr/bin/env python

# IMPORTS
import roslib
import rospy
import time
import tf
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import ceil
from blinky.srv import *
from blinky.msg import *
rospy.init_node('reinitializer')

# COLORS
BLACK = RGB(0, 0, 0)
CYAN = RGB(0, 255, 255)
GREEN = RGB(0, 255, 0)
ORANGE = RGB(255, 105, 0)
RED = RGB(255, 0, 0)
YELLOW = RGB(255, 200, 0)

# IMU READINGS
roll_angles = []
pitch_angles = []
yaw_angles = []

# COUNTDOWN STATE
is_counting_down = False
previous_blinky_step = 0

# TF LISTENER
listener = tf.TransformListener()


def set_planner(colors):
    """ Lights up planner blinkytape """
    try:
        rospy.wait_for_service('update_planner_lights')
        blinky_proxy = rospy.ServiceProxy('update_planner_lights', UpdatePlannerLights)
        result = blinky_proxy(colors)

        if result.success != 0:
            print 'UpdatePlannerLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def warning(state, frequency, color):
    """ Lights up blinkytape for warnings """
    try:
        rospy.wait_for_service('warning_lights')
        blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
        result = blinky_proxy([color], frequency, state)

        if result.success != 0:
            print 'WarningUpdateLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def countdown(time_counter, incomplete_color, complete_color, steps):
    """ Counts down on planner blinkytape """
    global is_counting_down, previous_blinky_step

    # SET FLAGS
    is_counting_down = True
    start_time = time.time()
    time_left = time_counter

    # COUNTDOWN
    while time_left > -0.1:
        # SET PLANNER BLINKY TAPE
        colors = []
        incomplete_blinky_steps = int(ceil(steps * time_left / time_counter))
        for blinkies in range(incomplete_blinky_steps):
            colors.append(incomplete_color)
        for blinkies in range(steps - incomplete_blinky_steps):
            colors.append(complete_color)
        set_planner(colors)

        # UPDATE FLAGS
        time_elapsed = time.time() - start_time
        time_left = time_counter - time_elapsed

        # PRINT
        if previous_blinky_step != incomplete_blinky_steps and incomplete_blinky_steps > 0:
            print incomplete_blinky_steps
            previous_blinky_step = incomplete_blinky_steps
        else:
            pass

    # RESET FLAGS
    is_counting_down = False


def imu_callback(pose):
    """ Callback for state estimation pose """
    global yaw_angles

    if is_counting_down:
        # DISREGARD MESSAGE, LOOK AT TF INSTEAD
        try:
            (trans,rot) = listener.lookupTransform(
                # from
                '/sensors/IMU_global_reference',
                # to
                '/robot/rotation_center',
                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        # GET ROLL, PITCH, YAW FROM QUATERNION
        (roll,pitch,yaw) = euler_from_quaternion(rot)
        roll_angles.append(roll)
        pitch_angles.append(pitch)
        yaw_angles.append(yaw)

    else:
        pass


def reinitialize_imu():
    """ Reinitialize IMU heading parameters """
    # AVERAGE OVER COUNTDOWN
    roll = sum(roll_angles)/len(roll_angles)
    pitch = sum(pitch_angles)/len(pitch_angles)
    yaw = sum(yaw_angles)/len(yaw_angles)

    # SET PARAMETERS
    rospy.set_param('/IMU/initial/roll',roll)
    rospy.set_param('/IMU/initial/pitch',pitch)
    rospy.set_param('/IMU/initial/yaw',yaw)

    # PRINT
    print 'AVERAGE IMU READINGS:'
    print 'ROLL: ', roll
    print 'PITCH:', pitch
    print 'YAW:  ', yaw


if __name__ == '__main__':
    if not rospy.is_shutdown():
        # GET TIMEOUT
        while not rospy.has_param('/countdown/timeout'):
            pass
        timeout = int(rospy.get_param('/countdown/timeout'))

        # HEADER
        print 'Recalibrating IMU in', timeout, 'seconds' if timeout != 1 else "second"

        # WARN BEFORE START
        time.sleep(timeout)
        print 'Ready...'
        set_planner([BLACK])
        warning(True, 1, YELLOW)
        time.sleep(5)
        print 'Set...'
        set_planner([YELLOW])
        warning(False, 0, BLACK)
        print 'Go.'
        time.sleep(1)

        # COUNTDOWN
        rospy.Subscriber('state_estimation/pose', PoseStamped, imu_callback)
        countdown(5, YELLOW, BLACK, 5)
        print 'Done.'
        reinitialize_imu()

        # WARN WHEN DONE
        warning(True, 2, GREEN)
        time.sleep(2.5)
        set_planner([GREEN])
        warning(False, 0, BLACK)

        # EXIT
        exit(0)
