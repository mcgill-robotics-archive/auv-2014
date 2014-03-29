#!/usr/bin/env python
#Created on Nov 16th, 2013

## @package central_app
#
#  Main file for McGill Robotics AUV Design Team testing User Interface
#
#  @author David Lavoie-Boutin

#bunch of import statements
#Ui declarations and GUI libraries
from pose_view_widget import PoseViewWidget
from no_imu import *
from low_battery_warning import*
from PyQt4 import QtCore, QtGui

import velocity_publisher  # custom modules for publishing cmd_vel and desired z position
import PS3Controller  # custom modules for acquiring ps3 input

from VARIABLES import *  # file containing all the shared variables and parameters

import sys
import signal

import rospy  # ros module for subscribing to topics
import pygame  # module top play the alarm

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from computer_vision.msg import VisibleObjectData


def stop_alarm():
    pygame.mixer.music.stop()


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

        QtCore.QObject.connect(self.battery_warning_ui.buttonBox, QtCore.SIGNAL("accepted()"), stop_alarm)

        self.battery_warning_ui.progressBar.setValue(misc_vars.low_battery_threshold/misc_vars.max_voltage*100)


## Main window class linking ROS with the UI and controllers
class CentralUi(QtGui.QMainWindow):
    ##Qt signal for battery empty alarm
    empty_battery_signal = QtCore.pyqtSignal()

    ## constructor of the main window for the ui
    #
    # it integrates the actual window as well as variables for the graphing areas and the video monitoring array
    # @param self The object pointer
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
        self.ui.label_13.setText("Depth")
        ## variable to enable or disable the keyboard monitoring
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
        self.ui.poseVizFrame.addWidget(self.pose_ui)

        # create initial data sets for imu, depth graphs
        ##dataset depth graph
        self.depth_data = []
        # fill the data sets with dummy data
        for i in range(0, misc_vars.length_plot, 1):
            self.depth_data.append(0)

        ##add the DEPTH GRAPH to the ui
        self.depth_graph = self.ui.imugraphics.addPlot(title="Depth")
        self.depth_curve = self.depth_graph.plot(pen="w")
        self.depth_graph.setXRange(0, misc_vars.length_plot)
        self.depth_graph.setYRange(0, misc_vars.depth_max)
        
        self.start_ros_subscriber()

        ## creates the timers to enable or disable the ps3 controller
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
        self.alarm_file = "/home/david/repo/McGill_RoboSub_2014/catkin_ws/src/front_end/scripts/resource/Ticktac.wav"

        # buttons connects
        # QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.set_controller_timer)
        QtCore.QObject.connect(self.ui.resSelect, QtCore.SIGNAL("currentIndexChanged(int)"), self.setVideoRes)

        # low battery connect
        self.empty_battery_signal.connect(self.open_low_battery_dialog)

        # controller timer connect
        QtCore.QObject.connect(self.ps3_timer, QtCore.SIGNAL("timeout()"), self.controller_update)
        QtCore.QObject.connect(self.key_timer, QtCore.SIGNAL("timeout()"), self.keyboard_update)

        ## A timer to redraw the GUI (video monitors)
        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_video_callback)
        self.redraw_timer.start(misc_vars.GUI_UPDATE_PERIOD)

    ##initiallize the ros subscribers
    #
    #initiallize the ros node and starts all the ros subscribers and maps each topic to the correct callback
    #
    #note that all the topic names are set in the file "VARIABLES.py"
    #@param self the object pointer
    def start_ros_subscriber(self):
        rospy.init_node('Front_End_UI', anonymous=True)
        rospy.Subscriber("/state_estimation/depth", Float32, self.depth_callback)
        rospy.Subscriber("/arduino/battery_voltage1", Float64, self.bat_1)
        rospy.Subscriber("/arduino/battery_voltage2", Float64, self.bat_2)
        rospy.Subscriber("/front_left_camera/image", Image, self.front_left_pre_callback)
        rospy.Subscriber("/front_right_camera/image", Image, self.front_right_pre_callback)
        rospy.Subscriber("/down_right_camera/image", Image, self.down_pre_callback)
        rospy.Subscriber("/front_cv/camera1", Image, self.front_post_left_callback)
        rospy.Subscriber("/front_cv/camera2", Image, self.front_post_right_callback)
        rospy.Subscriber("/down_cv/camera1", Image, self.down_post_callback)
        rospy.Subscriber('front_cv/data', VisibleObjectData, self.front_cv_data_callback)
        rospy.Subscriber('down_cv/data', VisibleObjectData, self.down_cv_data_callback)
        rospy.Subscriber('planner/task', String, self.planner_callback)
        rospy.Subscriber("/temp", Float32, self.temp_callback)

        #subscriber and callback for the 3d viz of pose data
        self.pose_ui.subscribe_topic('/state_estimation/pose')

    ##sets the maximum size of the video displays
    #
    #the selected resolution sets the maximal size to not over size the window
    #@param self the object pointer
    #@param data the data passed by the connect of the resolution combo box, is the index of the selected option
    def setVideoRes(self, data):
        #xga display
        if data == 0:
            self.ui.preLeft.setMaximumSize(QtCore.QSize(190, 170))
            self.ui.preRight.setMaximumSize(QtCore.QSize(190, 170))
            self.ui.preBottom.setMaximumSize(QtCore.QSize(190, 170))
            self.ui.postLeft.setMaximumSize(QtCore.QSize(190, 170))
            self.ui.postRight.setMaximumSize(QtCore.QSize(190, 170))
            self.ui.posBottom.setMaximumSize(QtCore.QSize(190, 170))
        #720p display
        elif data == 1:
            self.ui.preLeft.setMaximumSize(QtCore.QSize(220, 200))
            self.ui.preRight.setMaximumSize(QtCore.QSize(220, 200))
            self.ui.preBottom.setMaximumSize(QtCore.QSize(220, 200))
            self.ui.postLeft.setMaximumSize(QtCore.QSize(220, 200))
            self.ui.postRight.setMaximumSize(QtCore.QSize(220, 200))
            self.ui.posBottom.setMaximumSize(QtCore.QSize(220, 200))
        #1080p display
        elif data == 2:
            self.ui.preLeft.setMaximumSize(QtCore.QSize(320, 300))
            self.ui.preRight.setMaximumSize(QtCore.QSize(320, 300))
            self.ui.preBottom.setMaximumSize(QtCore.QSize(320, 300))
            self.ui.postLeft.setMaximumSize(QtCore.QSize(320, 300))
            self.ui.postRight.setMaximumSize(QtCore.QSize(320, 300))
            self.ui.posBottom.setMaximumSize(QtCore.QSize(320, 300))
        else:
            pass

    ##resize the sliders to fit the correct range of values
    def resizeSliders(self):
        self.ui.angularHorizantal.setRange(-1000*vel_vars.MAX_YAW_VEL, 1000*vel_vars.MAX_YAW_VEL)
        self.ui.linearVertical.setRange(-1000*vel_vars.MAX_LINEAR_VEL, 1000*vel_vars.MAX_LINEAR_VEL)
        self.ui.linearHorizantal.setRange(-1000*vel_vars.MAX_LINEAR_VEL, 1000*vel_vars.MAX_LINEAR_VEL)

    ## customisation of the key press class of the QWidget
    #  on key press,
    #  increments the variables contained in VARIABLES.py
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

            elif key == KeyMapping.IncreaseX:
                vel_vars.x_velocity -= vel_vars.MAX_LINEAR_VEL
            elif key == KeyMapping.DecreaseX:
                vel_vars.x_velocity -= -vel_vars.MAX_LINEAR_VEL

            elif key == KeyMapping.IncreaseY:
                vel_vars.y_velocity -= vel_vars.MAX_LINEAR_VEL
            elif key == KeyMapping.DecreaseY:
                vel_vars.y_velocity -= -vel_vars.MAX_LINEAR_VEL

    ##  procedure when the "Connect Controller" button is pressed
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
            if self.ps3.controller_isPresent:
                #self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/green.gif"))
                self.ps3_timer.start(misc_vars.controller_updateFrequency)
            else:
                #self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))
                pass
        # radio button KEYBOARD
        elif self.ui.keyboardControl.isChecked():
            self.ps3_timer.stop()
            self.keyboard_control = True
            #self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/yellow.gif"))
            self.key_timer.start(misc_vars.controller_updateFrequency)
        # radio button AUTONOMOUS
        elif self.ui.autonomousControl.isChecked():
            #QtCore.QTimer.singleShot(0, self.planner)
            self.keyboard_control = False
            self.ps3_timer.stop()
            self.key_timer.stop()
            velocity_publisher.velocity_publisher(vel_vars.x_velocity, -vel_vars.y_velocity, vel_vars.z_position, vel_vars.yaw_velocity, "planner/setPoints", 0)
            #self.ui.colourStatus.setPixmap(QtGui.QPixmap(":/Images/red.jpg"))

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
        self.ui.angularHorizantal.setValue(1000*vel_vars.yaw_velocity)

        self.ui.linearX.setText(str(vel_vars.x_velocity))
        self.ui.linearY.setText(str(vel_vars.y_velocity))
        self.ui.linearZ.setText(str(vel_vars.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularZ.setText(str(vel_vars.yaw_velocity))

        # publish to ros topic
        velocity_publisher.velocity_publisher(vel_vars.x_velocity, vel_vars.y_velocity, vel_vars.z_position, vel_vars.yaw_velocity, "planner/setPoints",1)

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
        self.ui.angularHorizantal.setValue(1000*vel_vars.yaw_velocity)

        self.ui.linearX.setText(str(vel_vars.x_velocity))
        self.ui.linearY.setText(str(vel_vars.y_velocity))
        self.ui.linearZ.setText(str(vel_vars.z_position))
        self.ui.angularX.setText(str(0))
        self.ui.angularZ.setText(str(vel_vars.yaw_velocity))

        # publish to ros topic
        velocity_publisher.velocity_publisher(vel_vars.x_velocity, -vel_vars.y_velocity, vel_vars.z_position, vel_vars.yaw_velocity, "planner/setPoints", 1)

    ## updates the data displayed by the depth graph
    #
    #appends the last value received by the depth_callback to the existing dataset and removes the first entry
    #@param self the object pointer
    #@param data_input data returned by the depth_callback
    def depth_graph_update(self, data_input):
        self.depth_data.append(data_input)
        self.depth_data.pop(0)
        self.depth_curve.setData(self.depth_data)
        if data_input > misc_vars.depth_max:
            misc_vars.depth_max = data_input
            self.depth_graph.setYRange(0, misc_vars.depth_max)

    def temp_callback(self, temp):
        self.ui.temp_ind.display(temp)

    ##
    #appends the last message recieved from planner to a textbox in screen
    #@param self the object pointer
    #@param string_data data passed by the ros callback
    def planner_callback(self, string_data):
        self.ui.logObject.append(string_data.data)

    def front_cv_data_callback(self, data):
        self.ui.front_pitch.setText(str(data.pitch_angle))
        self.ui.front_yaw.setText(str(data.yaw_angle))
        self.ui.front_x.setText(str(data.x_distance))
        self.ui.front_y.setText(str(data.y_distance))
        self.ui.front_z.setText(str(data.z_distance))

    def down_cv_data_callback(self, data):
        self.ui.down_pitch.setText(str(data.pitch_angle))
        self.ui.down_aw.setText(str(data.yaw_angle))
        self.ui.down_x.setText(str(data.x_distance))
        self.ui.down_y.setText(str(data.y_distance))
        self.ui.down_z.setText(str(data.z_distance))

    # VIDEO FRAME CALLBACKS
    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def front_left_pre_callback(self, data):
        try:
            self.left_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def front_right_pre_callback(self, data):
        try:
            self.right_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def down_pre_callback(self, data):
        try:
            self.bottom_pre_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def front_post_left_callback(self, data):
        try:
            self.left_post_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def front_post_right_callback(self, data):
        try:
            self.right_post_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ## when a frame is received, all the data is recorded in the appropriate variable
    #
    #takes the data received on the ros topic and records it to a variable
    #@param self the object pointer
    #@param data the data received by the subscriber
    def down_post_callback(self, data):
        try:
            self.bottom_post_image = data  # Save the ros image for processing by the display thread
        finally:
            pass

    ##updates the video frames displayed
    #
    # converts the image data from the ros message received
    #
    # converts it to a QPixmap and resets the displayed image
    #
    # does this for all screens
    #@param self the object pointer
    def redraw_video_callback(self):
        if self.left_pre_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_pre_image.data, self.left_pre_image.width, self.left_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.preLeft.setPixmap(image)

        else:
            self.ui.preLeft.setText("No video feed")

        if self.right_pre_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_pre_image.data, self.right_pre_image.width, self.right_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.preRight.setPixmap(image)
        else:
            self.ui.preRight.setText("No video feed")

        if self.bottom_pre_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_pre_image.data, self.bottom_pre_image.width, self.bottom_pre_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.preBottom.setPixmap(image)
        else:
            self.ui.preBottom.setText("No video feed")

        if self.left_post_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.left_post_image.data, self.left_post_image.width, self.left_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.postLeft.setPixmap(image)
        else:
            self.ui.postLeft.setText("No video feed")

        if self.right_post_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.right_post_image.data, self.right_post_image.width, self.right_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.postRight.setPixmap(image)
        else:
            self.ui.postRight.setText("No video feed")

        if self.bottom_post_image is not None:
            try:
                image = QtGui.QPixmap.fromImage(QtGui.QImage(self.bottom_post_image.data, self.bottom_post_image.width, self.bottom_post_image.height, QtGui.QImage.Format_RGB888))
            finally:
                pass

            self.ui.posBottom.setPixmap(image)
        else:
            self.ui.posBottom.setText("No video feed")

    # GRAPHS CALLBACKS
    ## callback for the depth topic
    #
    #calls to update the proper graph and passes the received data
    #@param self the object pointer
    #@param depth_data the data received by the subscriber
    def depth_callback(self, depth_data):
        self.depth_graph_update(depth_data.data)

    # LOW BATTERY ALARM
    ## callback for the battery voltage topic
    #
    # when voltage data is received, check if the voltage is ok,
    # if not, launch an alarm
    # @param self the object pointer
    # @param voltage_data data received by the subscriber
    def bat_1(self, voltage_data):
        self.ui.bat_lcd1.display(voltage_data)
        #if (not self.battery_empty) and voltage_data.data < misc_vars.low_battery_threshold:
        #    self.battery_empty = True
        #    self.empty_battery_signal.emit()
        #    self.play_alarm()
    def bat_2(self, voltage_data):
        self.ui.bat_lcd2.display(voltage_data)

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