##@package VARIABLES
#
# file grouping similar variables in classes
#since python does not have enums,
#
# also, these variables will be accessed by different python files therefore the need for a separate file
#file containing all variables shared between one or more files also contains parameters and constants
# used in the UI and the key mapping for the keyboard controller
#@author David Lavoie-Boutin
PI = 3.141592653
from PyQt4 import QtCore


##    variables shared by all controller for the cmd_vel and z_position
class vel_vars(object):
    yaw_velocity = 0
    x_velocity = 0
    y_velocity = 0
    z_position = 9
    z_position_step = 0.2

    MAX_YAW_VEL = 5
    MAX_LINEAR_VEL = 5


##    Here we define the keyboard map for our controller
class KeyMapping(object):
    YawLeft = QtCore.Qt.Key_J
    YawRight = QtCore.Qt.Key_L
    IncreaseDepth = QtCore.Qt.Key_R
    DecreaseDepth = QtCore.Qt.Key_W
    IncreaseX = QtCore.Qt.Key_F
    DecreaseX = QtCore.Qt.Key_S
    IncreaseY = QtCore.Qt.Key_E
    DecreaseY = QtCore.Qt.Key_D
    Surface = QtCore.Qt.Key_H


##   variables for the ros topic names
class ROS_Topics(object):
    battery_voltage = "battery_voltage"
    depth = "/state_estimation/depth"
    front_left_pre_topic = "/front_left_camera/image"
    front_right_pre_topic = "/front_right_camera/image"
    down_pre_topic = "/down_right_damera/image"
    front_left_post_topic = "/front_cv/camera1"
    front_right_post_topic = "/front_cv/camera2"
    down_post_topic = "/down_cv/camera1"
    simulator_pose = "/gazebo/model_states"
    imu_filtered = '/state_estimation/pose'
    vel_topic = '/planner/setPoints'
    front_cv_data = 'front_cv/data'
    down_cv_data = 'down_cv/data'
    planner_task = 'planner/task'


##    various parameters and constants
class misc_vars(object):
    controller_updateFrequency = 50
    low_battery_threshold = 2.0
    max_voltage = 24.0
    GUI_UPDATE_PERIOD = 10 #ms
    length_plot = 50
    depth_max = 0
