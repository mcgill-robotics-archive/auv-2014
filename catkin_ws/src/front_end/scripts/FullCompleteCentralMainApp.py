#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from CompleteFinal import *
from PyQt4 import QtCore, QtGui
import ps3_publisher
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
        self.ui = Ui_RoboticsMain()  # create the ui object
        self.ui.setupUi(self)
        self.controller_timer = QtCore.QTimer()
        self.ros_timer = QtCore.QTimer.singleShot(50, self.ros_subscriber)  # call the start of the ros subscriber
        self.ps3 = PS3Controller.PS3Controller()  # create the ps3 controller object

        #buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.setTimer)

        #timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.controllerUpdate)

        #create tuple list of checkboxes

        self.length_plot = 25

        #IMU PLOTS  #TODO assign to good graphicsView
        self.ui.imugraphics.addPlot()

        self.acc1 = self.ui.imugraphics.addPlot(title = "Accelerometer1")
        self.acc1_curve = self.acc1.plot(pen = "y")
        self.acc1.setXRange(0, self.length_plot)
        self.acc1_data = []
        for i in range(0, self.length_plot, 1):
            self.acc1_data.append(0)

        self.ui.imugraphics.nextRow()

        self.acc2 = self.ui.imugraphics.addPlot(title = "Accelerometer2")
        self.acc2_curve = self.acc2.plot(pen = "y")
        self.acc2.setXRange(0, self.length_plot)
        self.acc2_data = [0]
        for i in range(0, self.length_plot, 1):
            self.acc2_data.append(0)

        self.acc3 = self.ui.imugraphics.addPlot(title = "Accelerometer3")
        self.acc3_curve = self.acc3.plot(pen = "y")
        self.acc3.setXRange(0, self.length_plot)
        self.acc3_data = [0]
        for i in range(0, self.length_plot, 1):
            self.acc3_data.append(0)

        self.ui.imugraphics.nextRow()

        self.gy1 = self.ui.imugraphics.addPlot(title = "Gyro1")
        self.gy1_curve = self.gy1.plot(pen = "r")
        self.gy1.setXRange(0, self.length_plot)
        self.gy1_data = [0]
        for i in range(0, self.length_plot, 1):
            self.gy1_data.append(0)

        self.gy2 = self.ui.imugraphics.addPlot(title = "Gyro2")
        self.gy2_curve = self.gy2.plot(pen = "r")
        self.gy2.setXRange(0, self.length_plot)
        self.gy2_data = [0]
        for i in range(0, self.length_plot, 1):
            self.gy2_data.append(0)

        self.ui.imugraphics.nextRow()

        self.gy3 = self.ui.imugraphics.addPlot(title = "Gyro3")
        self.gy3_curve = self.gy3.plot(pen = "r")
        self.gy3.setXRange(0, self.length_plot)
        self.gy3_data = [0]
        for i in range(0, self.length_plot, 1):
            self.gy3_data.append(0)

        self.mag1 = self.ui.imugraphics.addPlot(title = "Magnetometer1")
        self.mag1_curve = self.mag1.plot(pen = "b")
        self.mag1.setXRange(0, self.length_plot)
        self.mag1_data = [0]
        for i in range(0, self.length_plot, 1):
            self.mag1_data.append(0)

        self.ui.imugraphics.nextRow()

        self.mag2 = self.ui.imugraphics.addPlot(title = "Magnetometer2")
        self.mag2_curve = self.mag2.plot(pen = "b")
        self.mag2.setXRange(0, self.length_plot)
        self.mag2_data = [0]
        for i in range(0, self.length_plot, 1):
            self.mag2_data.append(0)

        self.mag3 = self.ui.imugraphics.addPlot(title = "Magnetometer3")
        self.mag3_curve = self.mag3.plot(pen = "b")
        self.mag3.setXRange(0, self.length_plot)
        self.mag3_data = [0]
        for i in range(0, self.length_plot, 1):
            self.mag3_data.append(0)


        #PRESSURE GRAPH
        self.pressure_graph = self.ui.pressure.addPlot(title = "Pressure")
        self.pressure_curve = self.pressure_graph.plot(pen = "y")
        self.pressure_graph.setXRange(0, self.length_plot)
        self.pressure_data = [0]
        for i in range(0, self.length_plot, 1):
            self.pressure_data.append(0)

        #DEPTH GRAPH
        self.depth_graph = self.ui.depth.addPlot(title = "Depth")
        self.depth_curve = self.depth_graph.plot(pen = "y")
        self.depth_graph.setXRange(0, self.length_plot)
        self.depth_data = [0]
        for i in range(0, self.length_plot, 1):
            self.depth_data.append(0)




    def setTimer(self):
        """
        Start the timer
        """
        if self.ui.manualControl.isChecked():
            self.controller_timer.start(updateFrequency)
            if self.ps3.controller_name=="Sony PLAYSTATION(R)3 Controller":
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/green.gif"))
            else:
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/yellow.gif"))
        elif self.ui.autonomousControl.isChecked():
            self.controller_timer.stop()
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))
    def controllerUpdate(self):
        """
        slot for controller timer timeout
        """
        #update the state of the controller
        self.ps3.updateController()

        #react to the joysticks
        self.ui.linearVertical.setValue(1000*self.ps3.horizontal_front_speed)
        self.ui.linearHorizantal.setValue(-1000*self.ps3.horizontal_side_speed)
        self.ui.angularVertical.setValue(-1000*self.ps3.pitch_speed)
        self.ui.angularHorizantal.setValue(-1000*self.ps3.yaw_speed)

        self.ui.linearX.setText(str(self.ps3.horizontal_front_speed))
        self.ui.linearY.setText(str(self.ps3.horizontal_side_speed))
        self.ui.linearZ.setText(str(self.ps3.z_value))
        self.ui.angularX.setText(str(0))
        self.ui.angularY.setText(str(self.ps3.pitch_speed))
        self.ui.angularZ.setText((str(self.ps3.yaw_speed)))

# TODO : note to self, modified the axies for the demo, we need to set them back to the right ones!!!

        #publish to ros topic
        publisherText = ps3_publisher.ps3_publisher(self.ps3.horizontal_front_speed, -self.ps3.horizontal_side_speed, self.ps3.z_value, 0, self.ps3.yaw_speed, self.ps3.pitch_speed, 'gazebo/robot_twist')

        #display cmd_vel command to screen (on main ui not console)
        self.ui.logObject.append(publisherText)

    ###############
    #GRAPH UPDATER#
    ###############
    def acc1_update(self, data_input):
        self.acc1_data.append(data_input)
        self.acc1_data.pop(0)
        self.acc1_curve.setData(self.acc1_data)
    def acc2_update(self, data_input):
        self.acc2_data.append(data_input)
        self.acc2_data.pop(0)
        self.acc2_curve.setData(self.acc2_data)
    def acc3_update(self, data_input):
        self.acc3_data.append(data_input)
        self.acc3_data.pop(0)
        self.acc3_curve.setData(self.acc3_data)
    def pressure_graph_update(self, data_input):
        self.pressure_data.append(data_input)
        self.pressure_data.pop(0)
        self.pressure_curve.setData(self.pressure_data)
    def depth_graph_update(self, data_input):
        self.depth_data.append(data_input)
        self.depth_data.pop(0)
        self.depth_curve.setData(self.depth_data)
    def gy1_update(self, data_input):
        self.gy1_data.append(data_input)
        self.gy1_data.pop(0)
        self.gy1_curve.setData(self.gy1_data)
    def gy2_update(self, data_input):
        self.gy2_data.append(data_input)
        self.gy2_data.pop(0)
        self.gy2_curve.setData(self.gy2_data)
    def gy3_update(self, data_input):
        self.gy3_data.append(data_input)
        self.gy3_data.pop(0)
        self.gy3_curve.setData(self.gy3_data)
    def mag1_update(self, data_input):
        self.mag1_data.append(data_input)
        self.mag1_data.pop(0)
        self.mag1_curve.setData(self.mag1_data)
    def mag2_update(self, data_input):
        self.mag2_data.append(data_input)
        self.mag2_data.pop(0)
        self.mag2_curve.setData(self.mag2_data)
    def mag3_update(self, data_input):
        self.mag3_data.append(data_input)
        self.mag3_data.pop(0)
        self.mag3_curve.setData(self.mag3_data)

    ###############
    #ROS PUBLISHER#
    ###############
    def ros_subscriber(self):
        rospy.init_node('Front_End_UI', anonymous=True)
        rospy.Subscriber("pose", Pose, self.pose_callback)
        rospy.Subscriber("depth", Float32, self.depth_callback)
        rospy.Subscriber("pressure", Float32, self.pressure_callback)

    def pose_callback(self, pose_data):
        x = pose_data.orientation.x
        self.acc1_update(x)
        self.gy1_update(x)
        self.mag1_update(x)
        y = pose_data.orientation.y
        self.acc2_update(y)
        self.gy2_update(y)
        self.mag2_update(y)
        z = pose_data.orientation.z
        self.acc3_update(z)
        self.gy3_update(z)
        self.mag3_update(z)
        w = pose_data.orientation.w

    def depth_callback(self, depth_data):
        self.depth_graph_update(depth_data.data)

    def pressure_callback(self, pressure_data):
        self.pressure_graph_update(pressure_data.data)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = Main()
    AppWindow.show()
    sys.exit(app.exec_())
