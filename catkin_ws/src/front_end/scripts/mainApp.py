#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from UIFeedback import *
from PyQt4 import QtCore, QtGui
import ps3_publisher
import sys
import numpy
import frontEndSubscriber
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

updateFrequency = 50


class Main(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.controller_timer = QtCore.QTimer()
        #self.plot_timer = QtCore.QTimer()
        #self.plot_timer.start(50)

        self.ros_timer = QtCore.QTimer.singleShot(50, self.ros_subscriber)


        #create tuple list of checkboxes
        self.ps3 = PS3Controller.PS3Controller()
        self.checkboxList = (self.ui.select, self.ui.l3, self.ui.r3, self.ui.start, self.ui.l2, self.ui.r2, self.ui.l1, self.ui.r1, self.ui.triangle, self.ui.o, self.ui.x, self.ui.square)

        #FILL THE PLOT
        p2 = self.ui.graphicsView.addPlot(title="Multiple curves")
        p2.plot(numpy.random.normal(size=100), pen=(255,0,0))
        p2.plot(numpy.random.normal(size=100)+5, pen=(0,255,0))
        p2.plot(numpy.random.normal(size=100)+10, pen=(0,0,255))

        #new row
        self.ui.graphicsView.nextRow()

#UPDATING PLOT
        global med_curve_data, med_curve, updating_plot
        med_curve_data = [1]
        updating_plot = self.ui.graphicsView.addPlot(title = "Updating plot")
        med_curve = updating_plot.plot(pen = "r")




        #buttons
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.changeStatus, QtCore.SIGNAL("clicked()"), self.setTimer)

        #timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.controllerUpdate)
        #QtCore.QObject.connect(self.plot_timer, QtCore.SIGNAL("timeout()"), self.plot_update)

    def setTimer(self):
        """
        Start the timer
        """
        if self.ui.ManualRB.isChecked():
            self.controller_timer.start(updateFrequency)
        elif self.ui.AutonomusRB.isChecked():
            self.controller_timer.stop()

    def controllerUpdate(self):
        """
        slot for timer timeout
        """
        #update the state of the controller
        self.ps3.updateController()

        #react to the joysticks
        self.ui.verticalSlider.setValue(100*self.ps3.horizontal_front_speed)
        self.ui.horizontalSlider.setValue(-100*self.ps3.horizontal_side_speed)
        self.ui.verticalSlider_2.setValue(-100*self.ps3.pitch_speed)
        self.ui.horizontalSlider_3.setValue(-100*self.ps3.yaw_speed)
#TODO: change vertical speed(+ goes down)
        #react to the buttons
        buttonState = self.ps3.returnButtons()
        for i in range(0, len(buttonState), 1):
            if buttonState[i] == 1:
                self.checkboxList[i].setChecked(True)
            else:
                self.checkboxList[i].setChecked(False)

        #publish to ros topic
        publisherText = ps3_publisher.ps3_publisher(self.ps3.horizontal_front_speed, self.ps3.horizontal_side_speed, self.ps3.z_value, self.ps3.pitch_speed, self.ps3.yaw_speed, 0, 'turtle1/cmd_vel')

        #display cmd_vel command to screen (on main ui not console)
        self.ui.textField.append(publisherText)

    def plot_update(self, data_input):
        global med_curve, med_curve_data, updating_plot
        med_curve_data.append(data_input)
        med_curve.setData(med_curve_data)
        updating_plot.setXRange(len(med_curve_data)-100, len(med_curve_data))

    def ros_subscriber(self):
        rospy.init_node('frontEndSubscriber', anonymous=True)
        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("depth", Float32, self.depth_callback)
        rospy.Subscriber("pressure", Float32, self.pressure_callback)

    def pose_callback(self, pose_data):
        x = pose_data.orientation.x
        y = pose_data.orientation.y
        z = pose_data.orientation.z
        w = pose_data.orientation.w

        rospy.loginfo(rospy.get_name() + ": orientation : x, y, z, w: %f, %f, %f, %f", x, y, z, w)

    def depth_callback(self, depth_data):
        depth = depth_data.data
        self.plot_update(depth)
        #rospy.loginfo(rospy.get_name() + ": depth : %f", depth)

    def pressure_callback(self, pressure_data):
        pressure = pressure_data.data

        rospy.loginfo(rospy.get_name() + ": pressure : %f", pressure)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())