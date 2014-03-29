#!/usr/bin/env python

import signal
import sys
from PyQt4 import QtGui, QtCore
from pneumatics_ui import *
import rospy  # ros module for subscribing to topics
from arduino_msgs.msg import solenoid


class CentralUi(QtGui.QMainWindow):
    def __init__(self, parent=None):
        # build parent user interface
        super(CentralUi, self).__init__(parent)
        ## stores the ui object
        self.ui = Ui_Pnematic()

        self.ui.setupUi(self)

        rospy.init_node('pneumatic_tester', anonymous=False)
        self.pub = rospy.Publisher("/arduino/solenoid", solenoid)

        QtCore.QObject.connect(self.ui.tor1, QtCore.SIGNAL("clicked()"), self.torp1)
        QtCore.QObject.connect(self.ui.tor2, QtCore.SIGNAL("clicked()"), self.torp2)
        QtCore.QObject.connect(self.ui.mar1, QtCore.SIGNAL("clicked()"), self.mark1)
        QtCore.QObject.connect(self.ui.mar2, QtCore.SIGNAL("clicked()"), self.mark2)
        QtCore.QObject.connect(self.ui.grab1_open, QtCore.SIGNAL("clicked()"), self.grab1_open)
        QtCore.QObject.connect(self.ui.grab1_close, QtCore.SIGNAL("clicked()"), self.grab1_close)
        QtCore.QObject.connect(self.ui.grab2_close, QtCore.SIGNAL("clicked()"), self.grab2_open)
        QtCore.QObject.connect(self.ui.grab2_open, QtCore.SIGNAL("clicked()"), self.grab2_close)

    def torp1(self):
        msg = solenoid()
        msg.torpedo1 = True
        self.pub.publish(msg)

    def torp2(self):
        msg = solenoid()
        msg.torpedo2 = True
        self.pub.publish(msg)

    def mark1(self):
        msg = solenoid()
        msg.dropper1 = True
        self.pub.publish(msg)

    def mark2(self):
        msg = solenoid()
        msg.dropper2 = True
        self.pub.publish(msg)

    def grab1_close(self):
        msg = solenoid()
        msg.grabber1 = True
        self.pub.publish(msg)

    def grab2_close(self):
        msg = solenoid()
        msg.grabber2 = True
        self.pub.publish(msg)

    def grab1_open(self):
        msg = solenoid()
        msg.grabber1 = False
        self.pub.publish(msg)

    def grab2_open(self):
        msg = solenoid()
        msg.grabber2 = False
        self.pub.publish(msg)


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