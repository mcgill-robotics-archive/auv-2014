#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
from CompleteUI_declaration import *
from leak_warning import*
from PyQt4 import QtCore, QtGui
import ps3_data_publisher
import sys
import rospy
import pygame
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

updateFrequency = 50

class leak_warning_ui(QtGui.QDialog):
    def __init__(self, parent=None):
        super(leak_warning_ui, self).__init__(parent)
        self.leak_ui = Ui_warning()
        self.leak_ui.setupUi(self)

        QtCore.QObject.connect(self.leak_ui.buttonBox, QtCore.SIGNAL("accepted()"), self.stop_alarm)

    def stop_alarm(self):
        pygame.mixer.music.stop()



class central_ui(QtGui.QMainWindow):

    leaking_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):

        #build parent user interface
        super(central_ui, self).__init__(parent)
        self.ui = Ui_RoboticsMain()  # create the ui object
        self.ui.setupUi(self)
        self.controller_timer = QtCore.QTimer()
        self.ros_timer = QtCore.QTimer.singleShot(50, self.ros_subscriber)  # call the start of the ros subscriber
        self.ps3 = PS3Controller.PS3Controller()  # create the ps3 controller object
        self.leak_ui = leak_warning_ui(self)  # leak ui

        # place holder variable for internal leak status
        self.leak = False
        pygame.init()
        pygame.mixer.init()

        #TODO: change path to a machine specific path, I can't get this thing to work with a relative path
        self.alarm_file = "/home/david/repo/McGill_RoboSub_2014/catkin_ws/src/front_end/scripts/Ticktac.wav"

        #z destination publisher declaration
        self.zdes_pub = rospy.Publisher("zdes", Float64)

        #buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.set_ps3_timer)

        #leak connect
        self.leaking_signal.connect(self.open_dialog)

        #timer connect
        QtCore.QObject.connect(self.controller_timer, QtCore.SIGNAL("timeout()"), self.controller_update)

        self.length_plot = 25

        #IMU PLOTS

        # create initial data sets for imu, depth and pressure graphs
        self.acc1_data = []
        self.acc2_data = []
        self.acc3_data = []
        self.gy1_data = []
        self.gy2_data = []
        self.gy3_data = []
        self.mag1_data = []
        self.mag2_data = []
        self.mag3_data = []
        self.pressure_data = []
        self.depth_data = []

        for i in range(0, self.length_plot, 1):
            self.acc1_data.append(0)
            self.acc2_data.append(0)
            self.acc3_data.append(0)
            self.gy1_data.append(0)
            self.gy2_data.append(0)
            self.gy3_data.append(0)
            self.mag1_data.append(0)
            self.mag2_data.append(0)
            self.mag3_data.append(0)
            self.pressure_data.append(0)
            self.depth_data.append(0)

        #create and layout imu graphics
        self.ui.imugraphics.addPlot()

        self.acc1 = self.ui.imugraphics.addPlot(title="Accelerometer1")
        self.acc1_curve = self.acc1.plot(pen="y")
        self.acc1.setXRange(0, self.length_plot)

        self.ui.imugraphics.nextRow()

        self.acc2 = self.ui.imugraphics.addPlot(title="Accelerometer2")
        self.acc2_curve = self.acc2.plot(pen="y")
        self.acc2.setXRange(0, self.length_plot)

        self.acc3 = self.ui.imugraphics.addPlot(title="Accelerometer3")
        self.acc3_curve = self.acc3.plot(pen="y")
        self.acc3.setXRange(0, self.length_plot)

        self.ui.imugraphics.nextRow()

        self.gy1 = self.ui.imugraphics.addPlot(title="Gyro1")
        self.gy1_curve = self.gy1.plot(pen="r")
        self.gy1.setXRange(0, self.length_plot)

        self.gy2 = self.ui.imugraphics.addPlot(title="Gyro2")
        self.gy2_curve = self.gy2.plot(pen="r")
        self.gy2.setXRange(0, self.length_plot)

        self.ui.imugraphics.nextRow()

        self.gy3 = self.ui.imugraphics.addPlot(title="Gyro3")
        self.gy3_curve = self.gy3.plot(pen="r")
        self.gy3.setXRange(0, self.length_plot)

        self.mag1 = self.ui.imugraphics.addPlot(title="Magnetometer1")
        self.mag1_curve = self.mag1.plot(pen="b")
        self.mag1.setXRange(0, self.length_plot)

        self.ui.imugraphics.nextRow()

        self.mag2 = self.ui.imugraphics.addPlot(title="Magnetometer2")
        self.mag2_curve = self.mag2.plot(pen="b")
        self.mag2.setXRange(0, self.length_plot)

        self.mag3 = self.ui.imugraphics.addPlot(title="Magnetometer3")
        self.mag3_curve = self.mag3.plot(pen="b")
        self.mag3.setXRange(0, self.length_plot)

        #PRESSURE GRAPH
        self.pressure_graph = self.ui.pressure.addPlot(title="Pressure")
        self.pressure_curve = self.pressure_graph.plot(pen="y")
        self.pressure_graph.setXRange(0, self.length_plot)

        #DEPTH GRAPH
        self.depth_graph = self.ui.depth.addPlot(title="Depth")
        self.depth_curve = self.depth_graph.plot(pen="y")
        self.depth_graph.setXRange(0, self.length_plot)

    def set_ps3_timer(self):
        """
        Start the timer
        """
        if self.ui.manualControl.isChecked():
            if self.ps3.controller_name == "Sony PLAYSTATION(R)3 Controller":
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/green.gif"))
                self.controller_timer.start(updateFrequency)
            else:
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/yellow.gif"))
        elif self.ui.autonomousControl.isChecked():
            self.controller_timer.stop()
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))

    def controller_update(self):
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

# TODO : note to self, modified the axis for the demo, we need to set them back to the right ones!!!

        #publish to ros topic
        publisher_text = ps3_data_publisher.ps3_publisher(self.ps3.horizontal_front_speed, -self.ps3.horizontal_side_speed, self.ps3.z_value, 0, self.ps3.yaw_speed, self.ps3.pitch_speed, 'partial_cmd_vel')

        self.zdes_pub.publish(self.ps3.z_value)

        #display cmd_vel command to screen (on main ui not console)
        self.ui.logObject.append(publisher_text)

    ###############
    #GRAPH UPDATER#
    ###############

    def pressure_graph_update(self, data_input):
        self.pressure_data.append(data_input)
        self.pressure_data.pop(0)
        self.pressure_curve.setData(self.pressure_data)

    def depth_graph_update(self, data_input):
        self.depth_data.append(data_input)
        self.depth_data.pop(0)
        self.depth_curve.setData(self.depth_data)

    def imu_graph_updater(self, x, y, z, w):
        self.acc1_data.append(x)
        self.acc1_data.pop(0)
        self.acc1_curve.setData(self.acc1_data)

        self.acc2_data.append(y)
        self.acc2_data.pop(0)
        self.acc2_curve.setData(self.acc2_data)

        self.acc3_data.append(z)
        self.acc3_data.pop(0)
        self.acc3_curve.setData(self.acc3_data)

        self.gy1_data.append(y)
        self.gy1_data.pop(0)
        self.gy1_curve.setData(self.gy1_data)

        self.gy2_data.append(z)
        self.gy2_data.pop(0)
        self.gy2_curve.setData(self.gy2_data)

        self.gy3_data.append(w)
        self.gy3_data.pop(0)
        self.gy3_curve.setData(self.gy3_data)

        self.mag1_data.append(x)
        self.mag1_data.pop(0)
        self.mag1_curve.setData(self.mag1_data)

        self.mag2_data.append(y)
        self.mag2_data.pop(0)
        self.mag2_curve.setData(self.mag2_data)

        self.mag3_data.append(z)
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
        rospy.Subscriber("internal_pressure", Float64, self.internal_pressure_check)

    def pose_callback(self, pose_data):
        x = pose_data.orientation.x
        y = pose_data.orientation.y
        z = pose_data.orientation.z
        w = pose_data.orientation.w
        self.imu_graph_updater(x, y, z, w)

    def depth_callback(self, depth_data):
        self.depth_graph_update(depth_data.data)

    def pressure_callback(self, pressure_data):
        self.pressure_graph_update(pressure_data.data)

    def internal_pressure_check(self, internal_pressure_data):
        # TODO: create the pressure drop verification algorithm
        if (not self.leak) and internal_pressure_data.data<2:
            self.leak = True
            self.leaking_signal.emit()
            self.play_alarm()

        elif self.leak:
            self.zdes_pub.publish(0)

    def play_alarm(self):
        pygame.mixer.music.load(self.alarm_file)
        pygame.mixer.music.play(-1, 0)

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)


    def open_dialog(self):
        self.leak_ui.exec_()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = central_ui()
    AppWindow.show()
    sys.exit(app.exec_())