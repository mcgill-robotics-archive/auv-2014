##@package VARIABLES_conventions
#
# same as VARIABLES but following the naming conventions we set. Not the primary file yet since we still use the simulator
#and not all nodes have been changes.
#@author David Lavoie-Boutin
PI = 3.141592653
from PyQt4 import QtCore


##    variables shared by all controller for the cmd_vel and z_position
class vel_vars(object):
    yaw_velocity = 0
    x_velocity = 0
    y_velocity = 0
    depth_vel = 0
    z_position = 0
    z_position_step = 0.5

    MAX_YAW_VEL = 5
    MAX_LINEAR_VEL = 7


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


##    various parameters and constants
class misc_vars(object):
    controller_updateFrequency = 50
    low_battery_threshold = 21.0
    max_voltage = 24.0
    GUI_UPDATE_PERIOD = 10 #ms
    length_plot = 50
    depth_max = 0
    depth_min = 0
    max_temp=40
    CPU_max_temp = 90
    SSD_max_temp = 60
