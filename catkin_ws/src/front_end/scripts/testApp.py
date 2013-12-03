#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from CompleteFinale import *
from PyQt4 import QtCore, QtGui
import ps3_data_publisher
import sys
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

updateFrequency = 50

#TODO: alarm for internal pressure drop

class Main(QtGui.QMainWindow):
    def __init__(self, parent=None):
        #build parent user interface
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_MainWindow()  # create the ui object
        self.ui.setupUi(self)
        self.controller_timer = QtCore.QTimer()
        self.ros_timer = QtCore.QTimer.singleShot(50, self.ros_subscriber)  # call the start of the ros subscriber
        self.ps3 = PS3Controller.PS3Controller()  # create the ps3 controller object

        #buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.changeStatus, QtCore.SIGNAL("clicked()"), self.setTimer)

        #timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.controllerUpdate)

        #create tuple list of checkboxes
        self.checkboxList = (self.ui.select, self.ui.l3, self.ui.r3, self.ui.start, self.ui.l2, self.ui.r2, self.ui.l1, self.ui.r1, self.ui.triangle, self.ui.o, self.ui.x, self.ui.square)


        #IMU PLOTS
        self.acc1 = self.ui.graphicsView.addPlot(title = "Depth")
        self.acc1_curve = self.acc1.plot(pen = "y")
        self.acc1_data = [0]

        self.acc2 = self.ui.graphicsView.addPlot(title = "Pressure")
        self.acc2_curve = self.acc2.plot(pen = "y")
        self.acc2_data = [0]

        self.acc3 = self.ui.graphicsView.addPlot(title = "Updating plot")
        self.acc3_curve = self.acc3.plot(pen = "y")
        self.acc3_data = [0]

        #PRESSURE GRAPH  w
        self.pressure_graph = self.ui.graphicsView.addPlot(title = "Pressure")
        self.pressure_curve = self.pressure_graph.plot(pen = "y")
        self.pressure_data = [0]






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
        slot for controller timer timeout
        """
        #update the state of the controller
        self.ps3.updateController()

        #react to the joysticks
        self.ui.verticalSlider.setValue(100*self.ps3.horizontal_front_speed)
        self.ui.horizontalSlider.setValue(-100*self.ps3.horizontal_side_speed)
        self.ui.verticalSlider_2.setValue(-100*self.ps3.pitch_speed)
        self.ui.horizontalSlider_3.setValue(-100*self.ps3.yaw_speed)

        #react to the buttons
        buttonState = self.ps3.returnButtons()
        for i in range(0, len(buttonState), 1):
            if buttonState[i] == 1:
                self.checkboxList[i].setChecked(True)
            else:
                self.checkboxList[i].setChecked(False)

        #publish to ros topic
        publisherText = ps3_data_publisher.ps3_publisher(self.ps3.horizontal_front_speed, self.ps3.horizontal_side_speed, self.ps3.z_value, self.ps3.pitch_speed, self.ps3.yaw_speed, 0, 'turtle1/cmd_vel')

        #display cmd_vel command to screen (on main ui not console)
        self.ui.textField.append(publisherText)

    def acc1_update(self, data_input):
        self.acc1_data.append(data_input)
        self.acc1_curve.setData(self.acc1_data)
        self.acc1.setXRange(len(self.acc1_data)-100, len(self.acc1_data))

    def acc2_update(self, data_input):
        self.acc2_data.append(data_input)
        self.acc2_curve.setData(self.acc2_data)
        self.acc2.setXRange(len(self.acc2_data)-100, len(self.acc2_data))

    def acc3_update(self, data_input):
        self.acc3_data.append(data_input)
        self.acc3_curve.setData(self.acc3_data)
        self.acc3.setXRange(len(self.acc3_data)-100, len(self.acc3_data))

    def pressure_graph_update(self, data_input):
        self.pressure_data.append(data_input)
        self.pressure_curve.setData(self.pressure_data)
        self.pressure_graph.setXRange(len(self.pressure_data)-100, len(self.pressure_data))

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
        self.acc1_update(depth)

    def pressure_callback(self, pressure_data):
        pressure = pressure_data.data
        self.pressure_graph_update(pressure)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())