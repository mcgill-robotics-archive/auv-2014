#__author__ = 'david'
from  PyQt4 import QtCore

class vel_vars(object):

    #variables shared by all controller for the cmd_vel and z_position
    pitch_velocity = 0
    yaw_velocity = 0
    x_velocity = 0
    y_velocity = 0
    z_velocity = 0
    z_position = 0
    z_position_step = 0.5

    MAX_YAW_VEL = 1
    MAX_PITCH_VEL =1


class KeyMapping(object):
    # Here we define the keyboard map for our controller
    PitchForward = QtCore.Qt.Key_I
    PitchBackward = QtCore.Qt.Key_K
    YawLeft = QtCore.Qt.Key_J
    YawRight = QtCore.Qt.Key_L
    IncreaseDepth = QtCore.Qt.Key_U
    DecreaseDepth = QtCore.Qt.Key_O
    IncreaseX = QtCore.Qt.Key_S
    DecreaseX = QtCore.Qt.Key_F
    IncreaseY = QtCore.Qt.Key_E
    DecreaseY = QtCore.Qt.Key_D
    Surface = QtCore.Qt.Key_H

class ROS_Topics(object):
    battery_voltage="battery_voltage"
    pressure="pressure"
    depth="depth"
    left_pre_topic = "/my_robot/camera1/image_raw"
    imu_pose = "pose"
    partial_cmd_vel = '/gazebo/robot_twist'
    zdes = 'zdes'

class misc_vars(object):
    controller_updateFrequency = 50
    low_battery_threshold = 2.0
    max_voltage = 24.0
    GUI_UPDATE_PERIOD = 20 #ms