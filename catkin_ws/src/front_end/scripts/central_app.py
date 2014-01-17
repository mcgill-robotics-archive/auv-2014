#!/usr/bin/env python
#Created on Nov 16th, 2013

## @package central_app
#
#  Main file for McGill Robotics AUV Design Team testing User Interface

#  @author David Lavoie-Boutin

#bunch of import statements
#Ui declarations and GUI libraries

from pose_view_widget import PoseViewWidget
from CompleteUI_declaration import *
from low_battery_warning import*
from PyQt4 import QtCore, QtGui

import velocity_publisher  # custom modules for publishing cmd_vel and desired z position
import PS3Controller  # custom modules for acquiring ps3 input

from VARIABLES import *  # file containing all the shared variables and parameters

import sys
import rospy  # ros module for subscribing to topics
import pygame  # module top play the alarm
import numpy

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

from threading import Lock  # We need resource locking to handle synchronization between GUI thread and ROS callbacks

## Popup for low battery
#
# little class for displaying a popup when battery reaches critical levels
class BatteryWarningUi(QtGui.QDialog):
    ## The constructor
    #  Loads the ui declaration
    #  @param self The object pointer
    def __init__(self, parent=None):
        super(BatteryWarningUi, self).__init__(parent)
        ## store the ui object
        self.battery_warning_ui = Ui_warning()
        self.battery_warning_ui.setupUi(self)

        QtCore.QObject.connect(self.battery_warning_ui.buttonBox, QtCore.SIGNAL("accepted()"), self.stop_alarm)

        self.battery_warning_ui.progressBar.setValue(misc_vars.low_battery_threshold/misc_vars.max_voltage*100)
    ## Stops the alarm sound
    #  @param self The object pointer
    def stop_alarm(self):
        pygame.mixer.music.stop()

## Main window class linking ROS with the UI and controllers
class CentralUi(QtGui.QMainWindow):

    empty_battery_signal = QtCore.pyqtSignal()
    ## constructor of the main window for the ui
    #
    #  it integrates the actual window as well as variables for the graphing areas and the video monitoring array
    #  @param self The object pointer
    # build parent user interface
    # create the ui object
    # create the ps3 controller object
    # creates battery depleted ui
    def __init__(self, parent=None):
        # build parent user interface
        super(CentralUi, self).__init__(parent)
        ## stores the ui object
        self.ui = Ui_RoboticsMain()

        self.ui.setupUi(self)
        self.resizeSliders()

        
        ## dummy variable to enable or disable the keyboard monitoring
        self.keyboard_control = False

        ## Holds the left preprocessed image received from the sub and later processed by the GUI
        self.left_pre_image = None
        ## Holds the right preprocessed image received from the sub and later processed by the GUI
        self.right_pre_image = None
        ## Holds the bottom preprocessed image received from the sub and later processed by the GUI
        self.bottom_pre_image = None
        ## Holds the left post-processed image received from the sub and later processed by the GUI
        self.left_post_image = None
        ## Holds the right post-processed image received from the sub and later processed by the GUI
        self.right_post_image = None
        ## Holds the bottom post-processed image received from the sub and later processed by the GUI
        self.bottom_post_image = None

        ## Pose Visualiser widget
        self.pose_ui = PoseViewWidget(self)


        #TODO: change name of verticalLayout to prevent potential naming conflicts
        self.ui.verticalLayout.addWidget(self.pose_ui)

        # create initial data sets for imu, depth and pressure graphs
        ## the data set for accelerometer 1
        self.acc1_data = []
        ##data set accelerometer 2
        self.acc2_data = []
        ##dataset accelerometer 3
        self.acc3_data = []
        ##dataset gyroscope 1
        self.gy1_data = []
        ##dataset gyroscope 2
        self.gy2_data = []
        ##dataset gyroscope 3
        self.gy3_data = []
        ##dataset magnetmeter 1
        self.mag1_data = []
        ##dataset magnetmeter 2
        self.mag2_data = []
        ##dataset magnetmeter 3
        self.mag3_data = []
        ##dataset pressure graph
        self.pressure_data = []
        ##dataset depth graph
        self.depth_data = []

        # create the plots and start the ros subscribers
        self.create_plots()
        self.start_ros_subscriber()

        ## object to acquire lock on a thread to eliminate race conditions when updating the images
        self.image_lock = Lock()

        ## creates the timers to enable or disable the ps3
        self.ps3_timer = QtCore.QTimer()
        ## creates the timers to enable or disable the keyboard controllers
        self.key_timer = QtCore.QTimer()

        ## create the ps3 controller object
        self.ps3 = PS3Controller.PS3Controller()
        ## creates battery depleted ui
        self.warning_ui = BatteryWarningUi(self)

        ## place holder variable for internal battery status
        self.battery_empty = False
        
        # initiate pygame for the battery alarm
        pygame.init()
        pygame.mixer.init()


        #TODO: change path to a machine specific path, I can't get this thing to work with a relative path
        ## path to alarm sound
        self.alarm_file = "/home/david/repo/McGill_RoboSub_2014/catkin_ws/src/front_end/scripts/Ticktac.wav"

        # buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.set_controller_timer)

        # low battery connect
        self.empty_battery_signal.connect(self.open_low_battery_dialog)

        # controller timer connect
        QtCore.QObject.connect(self.ps3_timer, QtCore.SIGNAL("timeout()"), self.controller_update)
        QtCore.QObject.connect(self.key_timer, QtCore.SIGNAL("timeout()"), self.keyboard_update)

        ## A timer to redraw the GUI (video monitors)
        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_video_callback)
        self.redraw_timer.start(misc_vars.GUI_UPDATE_PERIOD)

    ##resize the sliders to fit the correct range of values
    def resizeSliders(self):
        self.ui.angularHorizantal.setRange(-1000*vel_vars.MAX_YAW_VEL, 1000*vel_vars.MAX_YAW_VEL)
        self.ui.angularVertical.setRange(-1000*vel_vars.MAX_PITCH_ANGLE, 1000*vel_vars.MAX_PITCH_ANGLE)
        self.ui.linearVertical.setRange(-1000*vel_vars.MAX_LINEAR_VEL, 1000*vel_vars.MAX_LINEAR_VEL)
        self.ui.linearHorizantal.setRange(-1000*vel_vars.MAX_LINEAR_VEL, 1000*vel_vars.MAX_LINEAR_VEL)

    ## customisation of the key press class of the QWidget
    #
    #  Increments the variables contained in VARIABLES.py
    #  @param self The object pointer
    #  @param event the data return by the Qt keyboard keypress signal
    def keyPressEvent(self, event):
        key = event.key()

        # If the key is not generated from an auto-repeating key
        if self.keyboard_control and not event.isAutoRepeat():
        # Handle the important cases first!
            if key == KeyMapping.Surface:
                vel_vars.z_position = 0
            else:
                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                if key == KeyMapping.YawLeft:
                    vel_vars.yaw_velocity += vel_vars.MAX_YAW_VEL
                elif key == KeyMapping.YawRight:
                    vel_vars.yaw_velocity += -vel_vars.MAX_YAW_VEL

                elif key == KeyMapping.PitchForward:
                    vel_vars.pitch_velocity += vel_vars.MAX_PITCH_ANGLE
                elif key == KeyMapping.PitchBackward:
                    vel_vars.pitch_velocity += -vel_vars.MAX_PITCH_ANGLE

                elif key == KeyMapping.IncreaseDepth:
                    vel_vars.z_position += vel_vars.z_position_step
                elif key == KeyMapping.DecreaseDepth:
                    vel_vars.z_position += -vel_vars.z_position_step

                elif key == KeyMapping.IncreaseX:
                    vel_vars.x_velocity += vel_vars.MAX_LINEAR_VEL
                elif key == KeyMapping.DecreaseX:
                    vel_vars.x_velocity += -vel_vars.MAX_LINEAR_VEL

                elif key == KeyMapping.IncreaseY:
                    vel_vars.y_velocity += vel_vars.MAX_LINEAR_VEL
                elif key == KeyMapping.DecreaseY:
                    vel_vars.y_velocity += -vel_vars.MAX_LINEAR_VEL

    ## customisation of the key release class of the QWidget
    #
    # Increments the variables contained in VARIABLES.py
    #  @param self The object pointer
    #  @param event the data return by the Qt keyboard keyrelease signal
    def keyReleaseEvent(self, event):

        key = event.key()

        # If the key is not generated from an auto-repeating key
        if self.keyboard_control and not event.isAutoRepeat():
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                vel_vars.yaw_velocity -= vel_vars.MAX_YAW_VEL
            elif key == KeyMapping.YawRight:
                vel_vars.yaw_velocity -= -vel_vars.MAX_YAW_VEL

            elif key == KeyMapping.PitchForward:
                vel_vars.pitch_velocity -= vel_vars.MAX_PITCH_ANGLE
            elif key == KeyMapping.PitchBackward:
                vel_vars.pitch_velocity -= -vel_vars.MAX_PITCH_ANGLE

            elif key == KeyMapping.IncreaseX:
                vel_vars.x_velocity -= vel_vars.MAX_LINEAR_VEL
            elif key == KeyMapping.DecreaseX:
                vel_vars.x_velocity -= -vel_vars.MAX_LINEAR_VEL

            elif key == KeyMapping.IncreaseY:
                vel_vars.y_velocity -= vel_vars.MAX_LINEAR_VEL
            elif key == KeyMapping.DecreaseY:
                vel_vars.y_velocity -= -vel_vars.MAX_LINEAR_VEL

    ##  procedure when the "Connect PS3 Controller" button is pressed
    #
    #   handles which timer to start for ps3 or keyboard_control or nothing
    #
    #   for the ps3 controller, checks which controller are connected, if only a ps3 controller is present, allows
    #   the initialisation and starts the ps3 timer,
    #   for the keyboard controller, simply start the keyboard timer
    #   @param self The object pointer
    def set_controller_timer(self):
        # radio button PS3
        if self.ui.manualControl.isChecked():
            self.keyboard_control=False
            self.key_timer.stop()
            # checks if the ps3 controller is present before starting the acquisition
            if self.ps3.controller_isPresent and self.ps3.controller_name == "Sony PLAYSTATION(R)3 Controller":
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/green.gif"))
                self.ps3_timer.start(misc_vars.controller_updateFrequency)
            else:
                self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))
        # radio button KEYBOARD
        elif self.ui.keyboardControl.isChecked():
            self.ps3_timer.stop()
            self.keyboard_control = True
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/yellow.gif"))
            self.key_timer.start(misc_vars.controller_updateFrequency)
        # radio button AUTONOMOUS
        elif self.ui.autonomousControl.isChecked():
            self.keyboard_control = False
            self.ps3_timer.stop()
            self.key_timer.stop()
            self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))

    ##  Method for the keyboard controller
    #
    #   activated when the key_timer times out,
    #   updates the ui with the values of the keyboard controller data and
    #   publishes to the correct topic
    #   @param self the object pointer
    def keyboard_update(self):
        # set ui
        self.ui.linearVertical.setValue(1000*vel_vars.x_velocity)
        self.ui.linearHorizantal.setValue(1000*vel_vars.y_velocity)
        self.ui.angularVertical.setValue(1000*vel_vars.pitch_velocity)
#        self.ui.angularVertical.setValue(vel_vars.pitch_velocity)
        self.ui.angularHorizantal.setValue(1000*vel_vars.yaw_velocity)

        self.ui.linearX.setText(str(vel_vars.x_velocity))
        self.ui.linearY.setText(str(vel_vars.y_velocity))
        self.ui.linearZ.setText(str(vel_vars.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularY.setText(str(vel_vars.pitch_velocity))
        self.ui.angularZ.setText(str(vel_vars.yaw_velocity))

        # publish to ros topic
        velocity_publisher.velocity_publisher(vel_vars.x_velocity, vel_vars.y_velocity, vel_vars.z_position, vel_vars.pitch_velocity, vel_vars.yaw_velocity, ROS_Topics.vel_topic)

    ## Method for the ps3 control
    #
    # activated when the ps3_timer times out
    #
    # updates the value of the ps3 controller data
    #
    # updates the ui with the values of the ps3 controller data
    #
    # publishes to the correct topic
    #  @param self the object pointer
    def controller_update(self):
        # update the state of the controller
        self.ps3.updateController()

        # set ui
        self.ui.linearVertical.setValue(1000*vel_vars.y_velocity)
        self.ui.linearHorizantal.setValue(1000*vel_vars.x_velocity)
        self.ui.angularVertical.setValue(1000*vel_vars.pitch_velocity)
        self.ui.angularHorizantal.setValue(1000*vel_vars.yaw_velocity)

        self.ui.linearX.setText(str(vel_vars.x_velocity))
        self.ui.linearY.setText(str(vel_vars.y_velocity))
        self.ui.linearZ.setText(str(vel_vars.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularY.setText(str(vel_vars.pitch_velocity))
        self.ui.angularZ.setText(str(vel_vars.yaw_velocity))

        # publish to ros topic
        velocity_publisher.velocity_publisher(vel_vars.x_velocity, -vel_vars.y_velocity, vel_vars.z_position, vel_vars.pitch_velocity, vel_vars.yaw_velocity, ROS_Topics.vel_topic)

    ## create and layout imu graphics
    #
    #  create all the gaphs in the imu area and adds a data set to the graph
    #  @param self the object pointer
    def create_plots(self):
        # fill the data sets with dummy data
        for i in range(0, misc_vars.length_plot, 1):
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

        # IMU PLOTS

        # create and layout imu graphics
        self.ui.imugraphics.addPlot()
        ##the accelerometer 1 plot
        self.acc1 = self.ui.imugraphics.addPlot(title="Accelerometer1")
        ##the accelerometer 1 curve
        self.acc1_curve = self.acc1.plot(pen="y")
        self.acc1.setXRange(0, misc_vars.length_plot)

        self.ui.imugraphics.nextRow()
        ##the accelerometer 2 plot
        self.acc2 = self.ui.imugraphics.addPlot(title="Accelerometer2")
        ##the accelerometer 2 curve
        self.acc2_curve = self.acc2.plot(pen="y")
        self.acc2.setXRange(0, misc_vars.length_plot)

        ##the accelerometer 3 plot
        self.acc3 = self.ui.imugraphics.addPlot(title="Accelerometer3")
        ##the accelerometer 3 curve
        self.acc3_curve = self.acc3.plot(pen="y")
        self.acc3.setXRange(0, misc_vars.length_plot)

        self.ui.imugraphics.nextRow()

        self.gy1 = self.ui.imugraphics.addPlot(title="Gyro1")
        self.gy1_curve = self.gy1.plot(pen="r")
        self.gy1.setXRange(0, misc_vars.length_plot)

        self.gy2 = self.ui.imugraphics.addPlot(title="Gyro2")
        self.gy2_curve = self.gy2.plot(pen="r")
        self.gy2.setXRange(0, misc_vars.length_plot)

        self.ui.imugraphics.nextRow()

        self.gy3 = self.ui.imugraphics.addPlot(title="Gyro3")
        self.gy3_curve = self.gy3.plot(pen="r")
        self.gy3.setXRange(0, misc_vars.length_plot)

        self.mag1 = self.ui.imugraphics.addPlot(title="Magnetometer1")
        self.mag1_curve = self.mag1.plot(pen="b")
        self.mag1.setXRange(0, misc_vars.length_plot)

        self.ui.imugraphics.nextRow()

        self.mag2 = self.ui.imugraphics.addPlot(title="Magnetometer2")
        self.mag2_curve = self.mag2.plot(pen="b")
        self.mag2.setXRange(0, misc_vars.length_plot)

        self.mag3 = self.ui.imugraphics.addPlot(title="Magnetometer3")
        self.mag3_curve = self.mag3.plot(pen="b")
        self.mag3.setXRange(0, misc_vars.length_plot)

        # PRESSURE GRAPH
        self.pressure_graph = self.ui.pressure.addPlot(title="Pressure")
        self.pressure_curve = self.pressure_graph.plot(pen="y")
        self.pressure_graph.setXRange(0, misc_vars.length_plot)

        # DEPTH GRAPH
        self.depth_graph = self.ui.depth.addPlot(title="Depth")
        self.depth_curve = self.depth_graph.plot(pen="y")
        self.depth_graph.setXRange(0, misc_vars.length_plot)

    # the following update the data sets displayed by each graph
    ## update the data displayed in the pressure graph
    #
    #appends the last value received by the pressure_callback the the existing dataset and removes the first entry
    #@param self the object pointer
    #@param data_input the data passed by the pressure_callback
    def pressure_graph_update(self, data_input):
        self.pressure_data.append(data_input)
        self.pressure_data.pop(0)
        self.pressure_curve.setData(self.pressure_data)

    ## updates the data displayed by the depth graph
    #
    #appends the last value received by the depth_callback to the existing dataset and removes the first entry
    #@param self the object pointer
    #@param data_input data returned by the depth_callback
    def depth_graph_update(self, data_input):
        self.depth_data.append(data_input)
        self.depth_data.pop(0)
        self.depth_curve.setData(self.depth_data)

    ## updates the data displayed by the IMU graph
    #
    #appends the last value received by the imu callback to the existing dataset and removes the first entry
    #@param self the object pointer
    #@param x data first data in the pose message returned by the imu callback
    #@param y data second data in the pose message returned by the imu callback
    #@param z data third data in the pose message returned by the imu callback
    #@param w data last data in the pose message returned by the imu callback
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

    ##initiallize the ros subscribers
    #
    #initiallize the ros node and starts all the ros subscribers and maps each topic to the correct callback
    #
    #note that all the topic names are set in the file "VARIABLES.py"
    #@param self the object pointer
    def start_ros_subscriber(self):
        rospy.init_node('Front_End_UI', anonymous=True)
        rospy.Subscriber(ROS_Topics.imu_raw, Pose, self.imu_callback)
        rospy.Subscriber(ROS_Topics.depth, Float32, self.depth_callback)
        rospy.Subscriber(ROS_Topics.pressure, Float32, self.pressure_callback)
        rospy.Subscriber(ROS_Topics.battery_voltage, Float64, self.battery_voltage_check)
        rospy.Subscriber(ROS_Topics.left_pre_topic, Image, self.pre_left_callback)
        rospy.Subscriber(ROS_Topics.right_pre_topic, Image, self.pre_right_callback)
        rospy.Subscriber(ROS_Topics.bottom_pre_topic, Image, self.pre_bottom_callback)
        rospy.Subscriber(ROS_Topics.left_post_topic, Image, self.post_left_callback)
        rospy.Subscriber(ROS_Topics.right_post_topic, Image, self.post_right_callback)
        rospy.Subscriber(ROS_Topics.bottom_post_topic, Image, self.post_bottom_callback)
        self.pose_ui.subscribe_topic("pose")
    # VIDEO FRAME CALLBACKS
    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def pre_left_callback(self, data):
        # We have some issues with locking between the GUI update thread
        # and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.left_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()
    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def pre_right_callback(self, data):
        # We have some issues with locking between the GUI update thread and
        # the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.right_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def pre_bottom_callback(self, data):
        # We have some issues with locking between the GUI update thread and
        # the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.bottom_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def post_left_callback(self, data):
        # We have some issues with locking between the GUI update thread
        # and the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.left_post_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def post_right_callback(self, data):
        # We have some issues with locking between the GUI update thread and
        # the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.right_post_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def post_bottom_callback(self, data):
        # We have some issues with locking between the GUI update thread and
        # the ROS messaging thread due to the size of the image, so we need to lock the resources
        self.image_lock.acquire()
        try:
            self.bottom_post_image = data  # Save the ros image for processing by the display thread
        finally:
            self.image_lock.release()
    ##updates the video frames displayed
    #
    # simply converts the image data from the ros message received
    #
    # converts it to a QPixmap and resets the displayed image
    #
    # does this for all screens
    #@param self the object pointer
    def redraw_video_callback(self):
        if self.left_pre_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_pre_image.data, self.left_pre_image.width, self.left_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.preLeft.setPixmap(image)
        else:
            self.ui.preLeft.setText("No video feed")

        if self.right_pre_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_pre_image.data, self.right_pre_image.width, self.right_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.preRight.setPixmap(image)
        else:
            self.ui.preRight.setText("No video feed")

        if self.bottom_pre_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_pre_image.data, self.bottom_pre_image.width, self.bottom_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.preBottom.setPixmap(image)
        else:
            self.ui.preBottom.setText("No video feed")

        if self.left_post_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_post_image.data, self.left_post_image.width, self.left_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.postLeft.setPixmap(image)
        else:
            self.ui.postLeft.setText("No video feed")

        if self.right_post_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_post_image.data, self.right_post_image.width, self.right_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.postRight.setPixmap(image)
        else:
            self.ui.postRight.setText("No video feed")

        if self.bottom_post_image is not None:
            self.image_lock.acquire()
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_post_image.data, self.bottom_post_image.width, self.bottom_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                self.image_lock.release()

            self.resize(image.width(), image.height())
            self.ui.posBottom.setPixmap(image)
        else:
            self.ui.posBottom.setText("No video feed")

    # GRAPHS CALLBACKS
    ## callback function for the imu topic
    #
    #stores the data received in variable
    #
    # calls to update the proper graph
    #@param self the object pointer
    #@param pose_data the pose message received by the subscriber
    def imu_callback(self, pose_data):
        x = pose_data.orientation.x
        y = pose_data.orientation.y
        z = pose_data.orientation.z
        w = pose_data.orientation.w
        self.imu_graph_updater(x, y, z, w)

    ## callback for the depth topic
    #
    #calls to update the proper graph and passes the received data
    #@param self the object pointer
    #@param depth_data the data received by the subscriber
    def depth_callback(self, depth_data):
        self.depth_graph_update(depth_data.data)

    ## callback for the pressure topic
    #
    #calls to update the proper graph and passes the received data
    #@param self the object pointer
    #@param pressure_data the data received by the subscriber
    def pressure_callback(self, pressure_data):
        self.pressure_graph_update(pressure_data.data)

    # LOW BATTERY ALARM
    ## callback for the battery voltage topic
    #
    # when voltage data is received, check if the voltage is ok,
    # if not, launch an alarm
    # @param self the object pointer
    # @param voltage_data data received by the subscriber
    def battery_voltage_check(self, voltage_data):

        if (not self.battery_empty) and voltage_data.data < misc_vars.low_battery_threshold:
            self.battery_empty = True
            self.empty_battery_signal.emit()
            self.play_alarm()
    ## start the alarm sound
    #
    #@param self the object pointer
    def play_alarm(self):
        pygame.mixer.music.load(self.alarm_file)
        pygame.mixer.music.play(-1, 0)

        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)
    ## launch the popup for the low battery
    #
    #@param self the object pointer
    def open_low_battery_dialog(self):
        self.warning_ui.exec_()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
