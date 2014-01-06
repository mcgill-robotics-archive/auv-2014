#!/usr/bin/env python
"""
Created on Nov 16th, 2013
@author : David Lavoie-Boutin
"""
import PS3Controller
#Ui declarations and GUI libraries
from CompleteUI_declaration import *
from low_battery_warning import*
from PyQt4 import QtCore, QtGui

import velocity_publisher

import sys
import rospy
import pygame
#ros message types
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

# We need to use resource locking to handle synchronization between GUI thread and ROS topic callbacks
from threading import Lock

controller_updateFrequency = 50
low_battery_threshold = 2.0
max_voltage = 24.0
GUI_UPDATE_PERIOD = 20 #ms


############
#ROS TOPICS#
############
battery_voltage="battery_voltage"
pressure="pressure"
depth="depth"
left_pre_topic = "/my_robot/camera1/image_raw"

# Here we define the keyboard map for our controller
class KeyMapping(object):
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

class battery_warning_ui(QtGui.QDialog):
    def __init__(self, parent=None):
        super(battery_warning_ui, self).__init__(parent)
        self.battery_warning_ui = Ui_warning()
        self.battery_warning_ui.setupUi(self)

        QtCore.QObject.connect(self.battery_warning_ui.buttonBox, QtCore.SIGNAL("accepted()"), self.stop_alarm)

        self.battery_warning_ui.progressBar.setValue(low_battery_threshold/max_voltage*100)

    def stop_alarm(self):
        pygame.mixer.music.stop()

class central_ui(QtGui.QMainWindow):

    empty_battery_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):

        #build parent user interface
        super(central_ui, self).__init__(parent)
        self.ui = Ui_RoboticsMain()  # create the ui object
        self.ui.setupUi(self)

        self.create_plots()
        self.start_ros_subscriber()

        self.pitch_velocity = 0
        self.yaw_velocity = 0
        self.x_velocity = 0
        self.y_velocity = 0
        self.z_velocity = 0
        self.z_position = 0

        self.keyboard_control = False

        # Holds the image frame received from the drone and later processed by the GUI
        self.left_pre_image = None
        self.right_pre_image = None
        self.bottom_pre_image = None
        self.left_post_image = None
        self.right_post_image = None
        self.bottom_post_image = None
        self.im_frame_array = (self.left_pre_image, self.right_pre_image, self.bottom_pre_image, self.left_post_image, self.right_post_image, self.bottom_post_image)

        self.imageLock = Lock()

        self.ps3_timer = QtCore.QTimer()
        self.keyTimer = QtCore.QTimer()

        self.ps3 = PS3Controller.PS3Controller()  # create the ps3 controller object
        self.warning_ui = battery_warning_ui(self)  # battery depleted ui

        # place holder variable for internal battery status
        self.battery_empty = False
        pygame.init()
        pygame.mixer.init()

        #TODO: change path to a machine specific path, I can't get this thing to work with a relative path
        self.alarm_file = "/home/david/repo/McGill_RoboSub_2014/catkin_ws/src/front_end/scripts/Ticktac.wav"

        #buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.set_controller_timer)

        #low battery connect
        self.empty_battery_signal.connect(self.open_low_battery_dialog)

        #controller timer connect
        QtCore.QObject.connect(self.ps3_timer, QtCore.SIGNAL("timeout()"), self.controller_update)
        #TODO: set timout function for keyboard
        QtCore.QObject.connect(self.keyTimer, QtCore.SIGNAL("timeout()"), self.keyboard_update)

        # A timer to redraw the GUI
        self.redrawTimer = QtCore.QTimer(self)
        self.redrawTimer.timeout.connect(self.RedrawVideoCallback)
        self.redrawTimer.start(GUI_UPDATE_PERIOD)

    # We add a keyboard handler to the DroneVideoDisplay to react to keypress
    def keyPressEvent(self, event):
        key = event.key()

        # If the key is not generated from an auto-repeating key
        if self.keyboard_control and not event.isAutoRepeat():
        # Handle the important cases first!
            if key == KeyMapping.Surface:
                self.z_position=0
            else:
                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += 1
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -1

                elif key == KeyMapping.PitchForward:
                    self.pitch_velocity += 1
                elif key == KeyMapping.PitchBackward:
                    self.pitch_velocity += -1

                elif key == KeyMapping.IncreaseDepth:
                    self.z_position += 1
                elif key == KeyMapping.DecreaseDepth:
                    self.z_position += -1

                elif key == KeyMapping.IncreaseX:
                    self.x_velocity += 1
                elif key == KeyMapping.DecreaseX:
                    self.x_velocity += -1

                elif key == KeyMapping.IncreaseY:
                    self.y_velocity += 1
                elif key == KeyMapping.DecreaseY:
                    self.y_velocity += -1

    def keyReleaseEvent(self, event):
        key = event.key()

        # If the key is not generated from an auto-repeating key
        if self.keyboard_control and not event.isAutoRepeat():
            # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= 1
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -1

            elif key == KeyMapping.PitchForward:
                self.pitch_velocity -= 1
            elif key == KeyMapping.PitchBackward:
                self.pitch_velocity -= -1

            elif key == KeyMapping.IncreaseX:
                self.x_velocity -= 1
            elif key == KeyMapping.DecreaseX:
                self.x_velocity -= -1

            elif key == KeyMapping.IncreaseY:
                self.y_velocity -= 1
            elif key == KeyMapping.DecreaseY:
                self.y_velocity -= -1

    def create_plots(self):
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

    def set_controller_timer(self):
        """
        Start the timer
        """
        if self.ui.manualControl.isChecked() and self.ps3.controller_isPresent:
            self.keyboard_control=False
            if self.ps3.controller_name == "Sony PLAYSTATION(R)3 Controller":
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/green.gif"))
                self.ps3_timer.start(controller_updateFrequency)
            else:
                self.keyboard_control=False
                self.ps3_timer.stop()
                self.keyTimer.stop()
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))

        elif self.ui.keyboardControl.isChecked():
            self.keyboard_control = True
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/yellow.gif"))
            self.keyTimer.start(controller_updateFrequency)

        elif self.ui.autonomousControl.isChecked():
            self.keyboard_control=False
            self.ps3_timer.stop()
            self.keyTimer.stop()
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))

    def keyboard_update(self):

        #react to the keys
        self.ui.linearVertical.setValue(100*self.y_velocity)
        self.ui.linearHorizantal.setValue(100*self.x_velocity)
        self.ui.angularVertical.setValue(100*self.pitch_velocity)
        self.ui.angularHorizantal.setValue(100*-self.yaw_velocity)

        self.ui.linearX.setText(str(self.x_velocity))
        self.ui.linearY.setText(str(self.y_velocity))
        self.ui.linearZ.setText(str(self.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularY.setText(str(self.pitch_velocity))
        self.ui.angularZ.setText((str(self.yaw_velocity)))

# TODO : note to self, modified the axis for the demo, we need to set them back to the right ones!!!

        #publish to ros topic
        velocity_publisher.velocity_publisher(self.x_velocity, self.y_velocity, self.z_position, self.pitch_velocity, self.yaw_velocity, 'partial_cmd_vel', 'zdes')

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
        self.ui.linearZ.setText(str(self.ps3.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularY.setText(str(self.ps3.pitch_speed))
        self.ui.angularZ.setText((str(self.ps3.yaw_speed)))

# TODO : note to self, modified the axis for the demo, we need to set them back to the right ones!!!

        #publish to ros topic
        velocity_publisher.velocity_publisher(self.ps3.horizontal_front_speed, -self.ps3.horizontal_side_speed, self.ps3.z_position, self.ps3.yaw_speed, self.ps3.pitch_speed, 'partial_cmd_vel', 'zdes')

        self.zdes_pub.publish(self.ps3.z_position)

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

    ################
    #ROS SUBSCRIBER#
    ################
    def start_ros_subscriber(self):
        #TODO: get real ros topic list
        rospy.init_node('Front_End_UI', anonymous=True)
        rospy.Subscriber("pose", Pose, self.imu_callback)
        rospy.Subscriber(depth, Float32, self.depth_callback)
        rospy.Subscriber(pressure, Float32, self.pressure_callback)
        rospy.Subscriber(battery_voltage, Float64, self.battery_voltage_check)
        rospy.Subscriber(left_pre_topic, Image, self.pre_left_callback)

    #VIDEO FRAME CALLBACKS
    def pre_left_callback(self, data):
        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.left_pre_image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()
    def pre_right_callback(self, data):
        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.right_pre_image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()
    def pre_bottom_callback(self, data):
        # We have some issues with locking between the GUI update thread and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.imageLock.acquire()
        try:
            self.bottom_pre_image = data # Save the ros image for processing by the display thread
        finally:
            self.imageLock.release()

    def RedrawVideoCallback(self):

        if self.left_pre_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_pre_image.data, self.left_pre_image.width, self.left_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.preLeft.setPixmap(image)
        else:
            self.ui.preLeft.setText("No video feed")

        if self.right_pre_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_post_image.data, self.right_pre_image.width, self.right_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.preRight.setPixmap(image)
        else:
            self.ui.preRight.setText("No video feed")

        if self.bottom_pre_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_post_image.data, self.bottom_pre_image.width, self.bottom_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.preBottom.setPixmap(image)
        else:
            self.ui.preBottom.setText("No video feed")

        if self.left_post_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_post_image.data, self.left_post_image, self.left_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.postLeft.setPixmap(image)
        else:
            self.ui.postLeft.setText("No video feed")

        if self.right_post_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_post_image.data, self.right_post_image, self.right_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.postRight.setPixmap(image)
        else:
            self.ui.postRight.setText("No video feed")

        if self.bottom_post_image is not None:
            self.imageLock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_post_image.data, self.bottom_post_image, self.bottom_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.imageLock.release()

            self.resize(image.width(),image.height())
            self.ui.posBottom.setPixmap(image)
        else:
            self.ui.posBottom.setText("No video feed")

    #GRAPHS CALLBACKS
    def imu_callback(self, pose_data):
        x = pose_data.orientation.x
        y = pose_data.orientation.y
        z = pose_data.orientation.z
        w = pose_data.orientation.w
        self.imu_graph_updater(x, y, z, w)
    def depth_callback(self, depth_data):
        self.depth_graph_update(depth_data.data)
    def pressure_callback(self, pressure_data):
        self.pressure_graph_update(pressure_data.data)

    #LOW BATTERY ALARM
    def battery_voltage_check(self, voltage_data):
        #TODO: set threshold for depleted battery
        if (not self.battery_empty) and voltage_data.data<low_battery_threshold:
            self.battery_empty = True
            self.empty_battery_signal.emit()
            self.play_alarm()
    def play_alarm(self):
        pygame.mixer.music.load(self.alarm_file)
        pygame.mixer.music.play(-1, 0)

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    def open_low_battery_dialog(self):
        self.warning_ui.exec_()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = central_ui()
    AppWindow.show()
    sys.exit(app.exec_())