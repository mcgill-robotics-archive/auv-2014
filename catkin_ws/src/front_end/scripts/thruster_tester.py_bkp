#!/usr/bin/env python

from thruster_control_ui import *
from PyQt4 import QtCore, QtGui

import sys
import os
import signal
from pygame import locals
import pygame
import rospy  # ros module for subscribing to topics

from controls.msg import motorCommands  # ros message types

os.environ["SDL_VIDEODRIVER"] = "dummy"

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
        self.controller_present = False
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            print "The controller is not connected"
            print "Shutting down the process..."
        elif pygame.joystick.get_count() == 1:
            print "One controller is connected"
            print "All set to go"
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
            self.controller_present = True
            self.ui.ps3_present.setChecked(True)
            print "The initialized Joystick is: " + self.controller.get_name()
        else:
            pass

        #timer for publisher
        self.value_update = QtCore.QTimer()
        QtCore.QObject.connect(self.value_update, QtCore.SIGNAL("timeout()"), self.publish_thrusters)
        self.value_update.start(200)

        #timer for controller
        self.controller_update = QtCore.QTimer()
        QtCore.QObject.connect(self.controller_update, QtCore.SIGNAL("timeout()"), self.updateController)
        if self.controller_present:
            print "Starting "
            self.controller_update.start(50)

        #init node
        rospy.init_node('manual_trust_mapper', anonymous=False)


        QtCore.QObject.connect(self.ui.x_force, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.x_bal, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.y_force, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.y_bal, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.z_force, QtCore.SIGNAL("valueChanged(int)"), self.z_force)
        QtCore.QObject.connect(self.ui.z_bal, QtCore.SIGNAL("valueChanged(int)"), self.z_force)

    def updateController(self):
        # If a changed occurred, the value will be updated; else the value will be the last one fetched.
        if self.controller_present:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:

                    if self.controller.get_button(3):  # left arrow
                        if self.ui.thruster_stop.isChecked():
                            self.ui.thruster_stop.setChecked(False)
                        else:
                            self.ui.thruster_stop.setChecked(True)
                            
                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    if self.controller.get_button(10):  # find l1
                        self.ui.fiel_thruste_1.setValue( -500*self.controller.get_axis(1))
                        self.ui.fiel_thruster_2.setValue( - 500*self.controller.get_axis(1))  
                        # ui.fiel_thruste_1 & x2 are the same, left front/back

                        if self.controller.get_axis(2) == 0:  # if not turning yaw (right x)
                            self.ui.fiel_thruster_3.setValue(500*self.controller.get_axis(0))
                            self.ui.fiel_thruster_4.setValue(-500*self.controller.get_axis(0))
                    if self.controller.get_button(11):  # find r1
                        self.ui.fiel_thruster_5.setValue(-500*self.controller.get_axis(3))
                        self.ui.fiel_thruster_6.setValue(-500*self.controller.get_axis(3))

                        self.ui.fiel_thruster_3.setValue(500*self.controller.get_axis(2))
                        self.ui.fiel_thruster_4.setValue(500*self.controller.get_axis(2))

    def x_force(self, data):
        if (self.ui.x_force.value() - self.ui.x_force.value() * self.ui.x_bal.value() / 100) > 500:
            self.ui.fiel_thruste_1.setValue(500)
        elif (self.ui.x_force.value() - self.ui.x_force.value() * self.ui.x_bal.value() / 100) < -500:
            self.ui.fiel_thruste_1.setValue(-500)
        else:
            self.ui.fiel_thruste_1.setValue(self.ui.x_force.value() - self.ui.x_force.value() * self.ui.x_bal.value() / 100)

        if (self.ui.x_force.value() + self.ui.x_force.value() * self.ui.x_bal.value() / 100) > 500:
            self.ui.fiel_thruster_2.setValue(500)
        elif (self.ui.x_force.value() + self.ui.x_force.value() * self.ui.x_bal.value() / 100) < -500:
            self.ui.fiel_thruster_2.setValue(-500)
        else:
            self.ui.fiel_thruster_2.setValue(self.ui.x_force.value() + self.ui.x_force.value() * self.ui.x_bal.value() / 100)

    def y_force(self, data):
        if (self.ui.y_force.value() - self.ui.y_force.value() * self.ui.y_bal.value() / 100) > 500:
            self.ui.fiel_thruster_3.setValue( 500)
        elif (self.ui.y_force.value() - self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.ui.fiel_thruster_3.setValue( -500)
        else:
            self.ui.fiel_thruster_3.setValue( self.ui.y_force.value() - self.ui.y_force.value() * self.ui.y_bal.value() / 100)

        if (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100) > 500:
            self.ui.fiel_thruster_4.setValue( -500)
        elif (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.ui.fiel_thruster_4.setValue( 500)
        else:
            self.ui.fiel_thruster_4.setValue(- (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100))

    def z_force(self, data):
        if (self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.ui.fiel_thruster_5.setValue( 500)
        elif (self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.ui.fiel_thruster_5.setValue( -500)
        else:
            self.ui.fiel_thruster_5.setValue( self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100)

        if (self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.ui.fiel_thruster_6.setValue( 500)
        elif (self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.ui.fiel_thruster_6.setValue( -500)
        else:
            self.ui.fiel_thruster_6.setValue( self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100)

    def publish_thrusters(self):
        vel_pub = rospy.Publisher("/electrical_interface/motor", motorCommands)

        msg = motorCommands()
        if not self.ui.thruster_stop.isChecked():
            msg.cmd_surge_starboard = self.ui.fiel_thruste_1.value()
            msg.cmd_surge_port = self.ui.fiel_thruster_2.value()
            msg.cmd_sway_bow = self.ui.fiel_thruster_3.value()
            msg.cmd_sway_stern = self.ui.fiel_thruster_4.value()
            msg.cmd_heave_bow = self.ui.fiel_thruster_5.value()
            msg.cmd_heave_stern = self.ui.fiel_thruster_6.value()
        else:
            msg.cmd_surge_starboard = 0
            msg.cmd_surge_port = 0
            msg.cmd_sway_bow = 0
            msg.cmd_sway_stern = 0
            msg.cmd_heave_bow = 0
            msg.cmd_heave_stern = 0

        vel_pub.publish(msg)


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

