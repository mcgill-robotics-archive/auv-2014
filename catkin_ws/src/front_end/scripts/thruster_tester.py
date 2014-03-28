#!/usr/bin/env python
#Created on Nov 16th, 2013

## @package central_app
#
#  Main file for McGill Robotics AUV Design Team testing User Interface
#
#  @author David Lavoie-Boutin

#bunch of import statements
#Ui declarations and GUI libraries

import signal

from thruster_control_ui import *
from PyQt4 import QtCore, QtGui

import sys

import rospy  # ros module for subscribing to topics

from controls.msg import motorCommands  # ros message types


## Main window class linking ROS with the UI and controllers
class CentralUi(QtGui.QMainWindow):
    ##Qt signal for battery empty alarm
    empty_battery_signal = QtCore.pyqtSignal()

    ## constructor of the main window for the ui
    #
    # it integrates the actual window
    # @param self The object pointer
    # build parent user interface
    # create the ui object
    def __init__(self, parent=None):
        # build parent user interface
        super(CentralUi, self).__init__(parent)
        ## stores the ui object
        self.ui = Ui_Thrust_Ui()

        self.ui.setupUi(self)

        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0
        self.z1 = 0
        self.z2 = 0

        # buttons connects
        QtCore.QObject.connect(self.ui.mode_selector, QtCore.SIGNAL("currentIndexChanged(int)"), self.change_mode)

        self.value_update = QtCore.QTimer()
        QtCore.QObject.connect(self.value_update, QtCore.SIGNAL("timeout()"), self.publish_thrusters)
        self.value_update.start(50)

        rospy.init_node('manual_trust_mapper', anonymous=False)

        QtCore.QObject.connect(self.ui.fiel_thruste_1, QtCore.SIGNAL("valueChanged(int)"), self.setX1)
        QtCore.QObject.connect(self.ui.fiel_thruster_2, QtCore.SIGNAL("valueChanged(int)"), self.setX2)
        QtCore.QObject.connect(self.ui.fiel_thruster_3, QtCore.SIGNAL("valueChanged(int)"), self.setY1)
        QtCore.QObject.connect(self.ui.fiel_thruster_4, QtCore.SIGNAL("valueChanged(int)"), self.setY2)
        QtCore.QObject.connect(self.ui.fiel_thruster_5, QtCore.SIGNAL("valueChanged(int)"), self.setZ1)
        QtCore.QObject.connect(self.ui.fiel_thruster_6, QtCore.SIGNAL("valueChanged(int)"), self.setZ2)
        QtCore.QObject.connect(self.ui.x_force, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.x_bal, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.y_force, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.y_bal, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.z_force, QtCore.SIGNAL("valueChanged(int)"), self.z_force)
        QtCore.QObject.connect(self.ui.z_bal, QtCore.SIGNAL("valueChanged(int)"), self.z_force)

    def setX1(self, number):
        self.x1 = number

    def setX2(self, number):
        self.x2 = number

    def setY1(self, number):
        self.y1 = number

    def setY2(self, number):
        self.y2 = number

    def setZ1(self, number):
        self.z1 = number

    def setZ2(self, number):
        self.z2 = number

    def x_force(self, data):
        if (5 * self.ui.x_force.value() - 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100) > 500:
            self.x1 = 500
        elif (5 * self.ui.x_force.value() - 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100) < -500:
            self.x1 = -500
        else:
            self.x1 = 5 * self.ui.x_force.value() - 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100

        if (5 * self.ui.x_force.value() + 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100) > 500:
            self.x2 = 500
        elif (5 * self.ui.x_force.value() + 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100) < -500:
            self.x2 = -500
        else:
            self.x2 = 5 * self.ui.x_force.value() + 5 * self.ui.x_force.value() * self.ui.x_bal.value() / 100

    def y_force(self, data):
        if (5 * self.ui.y_force.value() - 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100) > 500:
            self.y1 = 500
        elif (5 * self.ui.y_force.value() - 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.y1 = -500
        else:
            self.y1 = 5 * self.ui.y_force.value() - 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100

        if (5 * self.ui.y_force.value() + 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100) > 500:
            self.y2 = 500
        elif (5 * self.ui.y_force.value() + 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.y2 = -500
        else:
            self.y2 = 5 * self.ui.y_force.value() + 5 * self.ui.y_force.value() * self.ui.y_bal.value() / 100

    def z_force(self, data):
        if (5 * self.ui.z_force.value() - 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.z1 = 500
        elif (5 * self.ui.z_force.value() - 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.z1 = -500
        else:
            self.z1 = 5 * self.ui.z_force.value() - 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100

        if (5 * self.ui.z_force.value() + 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.z2 = 500
        elif (5 * self.ui.z_force.value() + 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.z2 = -500
        else:
            self.z2 = 5 * self.ui.z_force.value() + 5 * self.ui.z_force.value() * self.ui.z_bal.value() / 100


    def publish_thrusters(self):
        vel_pub = rospy.Publisher("/controls/motor_commands", motorCommands)

        msg = motorCommands()

        msg.cmd_x1 = self.x1
        msg.cmd_x2 = self.x2
        msg.cmd_y1 = self.y1
        msg.cmd_y2 = self.y2
        msg.cmd_z1 = self.z1
        msg.cmd_z2 = self.z2

        vel_pub.publish(msg)

    def change_mode(self, mode):
        if mode == 0:  # individual control
            self.ui.x_bal.setEnabled(False)
            self.ui.x_force.setEnabled(False)
            self.ui.y_bal.setEnabled(False)
            self.ui.y_force.setEnabled(False)
            self.ui.z_bal.setEnabled(False)
            self.ui.z_force.setEnabled(False)
            self.ui.slider_thruster_11.setEnabled(False)
            self.ui.slider_thruster_12.setEnabled(False)
            self.ui.slider_thruster_13.setEnabled(False)
            self.ui.slider_thruster_14.setEnabled(False)
            self.ui.slider_thruster_15.setEnabled(False)
            self.ui.slider_thruster_16.setEnabled(False)
            self.ui.fiel_thruste_1.setEnabled(True)
            self.ui.fiel_thruster_2.setEnabled(True)
            self.ui.fiel_thruster_3.setEnabled(True)
            self.ui.fiel_thruster_4.setEnabled(True)
            self.ui.fiel_thruster_5.setEnabled(True)
            self.ui.fiel_thruster_6.setEnabled(True)
            self.ui.slider_thruster_1.setEnabled(True)
            self.ui.slider_thruster_2.setEnabled(True)
            self.ui.slider_thruster_3.setEnabled(True)
            self.ui.slider_thruster_4.setEnabled(True)
            self.ui.slider_thruster_5.setEnabled(True)
            self.ui.slider_thruster_6.setEnabled(True)
        elif mode == 1:  # group control
            self.ui.x_bal.setEnabled(True)
            self.ui.x_force.setEnabled(True)
            self.ui.y_bal.setEnabled(True)
            self.ui.y_force.setEnabled(True)
            self.ui.z_bal.setEnabled(True)
            self.ui.z_force.setEnabled(True)
            self.ui.slider_thruster_11.setEnabled(True)
            self.ui.slider_thruster_12.setEnabled(True)
            self.ui.slider_thruster_13.setEnabled(True)
            self.ui.slider_thruster_14.setEnabled(True)
            self.ui.slider_thruster_15.setEnabled(True)
            self.ui.slider_thruster_16.setEnabled(True)
            self.ui.fiel_thruste_1.setEnabled(False)
            self.ui.fiel_thruster_2.setEnabled(False)
            self.ui.fiel_thruster_3.setEnabled(False)
            self.ui.fiel_thruster_4.setEnabled(False)
            self.ui.fiel_thruster_5.setEnabled(False)
            self.ui.fiel_thruster_6.setEnabled(False)
            self.ui.slider_thruster_1.setEnabled(False)
            self.ui.slider_thruster_2.setEnabled(False)
            self.ui.slider_thruster_3.setEnabled(False)
            self.ui.slider_thruster_4.setEnabled(False)
            self.ui.slider_thruster_5.setEnabled(False)
            self.ui.slider_thruster_6.setEnabled(False)


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    #if QtGui.QMessageBox.question(None, '', "Are you sure you want to quit?",
    #                              QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
    #                              QtGui.QMessageBox.No) == QtGui.QMessageBox.Yes:
    QtGui.QApplication.quit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    app = QtGui.QApplication(sys.argv)
    timer = QtCore.QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    # Your code here.
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())

