#!/usr/bin/env python

# IMPORTS
import roslib
import rospy
import time
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from math import ceil
from blinky.srv import *
from blinky.msg import *
rospy.init_node('initializer')

# COLORS
BLACK = RGB(0, 0, 0)
CYAN = RGB(0, 255, 255)
GREEN = RGB(0, 255, 0)
ORANGE = RGB(255, 105, 0)
RED = RGB(255, 0, 0)
WHITE = RGB(255, 255, 255)
YELLOW = RGB(255, 200, 0)
original_colors = []
got_original_colors = False

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
    """ Lights up planner BlinkyTape """
    try:
        rospy.wait_for_service('update_planner_lights')
        blinky_proxy = rospy.ServiceProxy('update_planner_lights', UpdatePlannerLights)
        result = blinky_proxy(colors)

        if result.success != 0:
            print 'UpdatePlannerLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def warning(state, frequency, color):
    """ Lights up BlinkyTape for warnings """
    try:
        rospy.wait_for_service('warning_lights')
        blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
        result = blinky_proxy([color], frequency, state)

        if result.success != 0:
            print 'WarningUpdateLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def countdown(time_counter, incomplete_color, complete_color, steps):
    """ Counts down on planner BlinkyTape """
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
            rospy.logwarn('%d blinkies left', incomplete_blinky_steps)
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
                '/sensors/raw/IMU_global_reference',
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


def planner_callback(data):
    """ Gets colors currently displayed on the planner BlinkyTape """
    global original_colors, got_original_colors

    if not got_original_colors:
        original_colors = data.colors
        got_original_colors = True
    else:
        pass


def reinitialize_imu(average):
    """ Reinitializes IMU heading parameters """
    # MEAN OR MEDIAN OVER COUNTDOWN
    if mean:
        roll = sum(roll_angles)/len(roll_angles)
        pitch = sum(pitch_angles)/len(pitch_angles)
        yaw = sum(yaw_angles)/len(yaw_angles)
    else:
        roll = float(np.median(roll_angles))
        pitch = float(np.median(pitch_angles))
        yaw = float(np.median(yaw_angles))

    # SET PARAMETERS
    rospy.set_param('/IMU/initial/roll',roll)
    rospy.set_param('/IMU/initial/pitch',pitch)
    rospy.set_param('/IMU/initial/yaw',yaw)
    rospy.set_param('/IMU/reinitialized',True)

    # LOG
    rospy.logwarn('FILTERED %d DATA POINTS', len(roll_angles))
    rospy.logwarn('INITIAL ROLL: %f', roll)
    rospy.logwarn('INITIAL PITCH:%f', pitch)
    rospy.logwarn('INITIAL YAW:  %f', yaw)


def ready_to_go(go_signal):
    """ Checks if planner is running and computer is untethered """
    connection = open('/sys/class/net/eth0/operstate').read().rstrip()
    yes = go_signal and connection == "down"

    return yes


if __name__ == '__main__':
    if not rospy.is_shutdown():
        # GET TIMEOUT
        while not rospy.has_param('/countdown/'):
            pass
        mean = rospy.get_param('/countdown/mean')
        timeout = int(rospy.get_param('/countdown/timeout'))
        timer = int(rospy.get_param('/countdown/timer'))
        go_signal = rospy.get_param('/countdown/go')

        # HEADER
        rospy.logwarn('Initializing IMU in %d sec', timeout)
        rospy.Subscriber('original_planner_colors', RGBArray, planner_callback)

        # WARN BEFORE START
        time.sleep(timeout)
        rospy.logwarn('Ready...')
        while not got_original_colors:
            pass
        set_planner([BLACK])
        rospy.logwarn('Set..')
        warning(True, 1, YELLOW)
        time.sleep(5.25)
        warning(False, 0, BLACK)
        rospy.logwarn('Go.')
        time.sleep(0.5)

        # COUNTDOWN
        rospy.Subscriber('state_estimation/pose', PoseStamped, imu_callback)
        countdown(timer, YELLOW, BLACK, 5)
        rospy.logwarn('DONE')
        reinitialize_imu(mean)

        # WARN WHEN DONE
        set_planner([BLACK])
        warning(True, 2, GREEN)
        time.sleep(2.2)
        warning(False, 0, BLACK)
        time.sleep(0.5)
        if go_signal:
            set_planner([CYAN])
        else:
            set_planner(original_colors)

        # GO IF ASKED AND UNTETHERED
        if ready_to_go(go_signal):
            rospy.logwarn("GO ASIMOV GO")
            rospy.set_param('/go', 1)

        # EXIT
        exit(0)
