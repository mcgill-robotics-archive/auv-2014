##@package VARIABLES
#
# file grouping similar variables in classes
#since python does not have enums,
#
# also, these variables will be accessed by different python files therefore the need for a separate file
#file containing all variables shared between one or more files also contains parameters and constants used in the UI and the key mapping for the keyboard controller
#@author David Lavoie-Boutin
PI = 3.141592653
from PyQt4 import QtCore
##    variables shared by all controller for the cmd_vel and z_position
class vel_vars(object):
    pitch_velocity = 0
    yaw_velocity = 0
    x_velocity = 0
    y_velocity = 0
    z_position = 0
    z_position_step = 0.5

    MAX_YAW_VEL = 10
    MAX_PITCH_ANGLE = PI/4
    MAX_LINEAR_VEL = 1

##    Here we define the keyboard map for our controller
class KeyMapping(object):
    PitchForward = QtCore.Qt.Key_I
    PitchBackward = QtCore.Qt.Key_K
    YawLeft = QtCore.Qt.Key_J
    YawRight = QtCore.Qt.Key_L
    IncreaseDepth = QtCore.Qt.Key_R
    DecreaseDepth = QtCore.Qt.Key_W
    IncreaseX = QtCore.Qt.Key_E
    DecreaseX = QtCore.Qt.Key_D
    IncreaseY = QtCore.Qt.Key_F
    DecreaseY = QtCore.Qt.Key_S
    Surface = QtCore.Qt.Key_H

##   variables for the ros topic names
class ROS_Topics(object):
    battery_voltage = "battery_voltage"
    pressure = "pressure"
    depth = "depth"
    left_pre_topic = "/simulator/camera1/image_raw"
    right_pre_topic = "/simulator/camera2/image_raw"
    bottom_pre_topic = "/simulator/camera3/image_raw"
    left_post_topic = "/front_cv_camera1"
    right_post_topic = "/front_cv_camera2"
    bottom_post_topic = "/down_cv_camera"
    imu_raw = 'pose'
    imu_filtered='pose'
    vel_topic = '/setPoints'

##    various parameters and constants
class misc_vars(object):
    controller_updateFrequency = 50
    low_battery_threshold = 2.0
    max_voltage = 24.0
    GUI_UPDATE_PERIOD = 1000 #ms
    length_plot = 25
