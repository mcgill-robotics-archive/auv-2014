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
from Battery_warning_popup import*
from PyQt4 import QtCore, QtGui

import velocity_publisher  # custom modules for publishing cmd_vel
import PS3Controller_central  # custom modules for acquiring ps3 input

from VARIABLES import *  # file containing all the shared variables and parameters

import sys
import signal

import rospy  # ros module for subscribing to topics
import pygame  # module top play the alarm

from std_msgs.msg import String  # ros message types
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from computer_vision.msg import VisibleObjectData
from controls.msg import motorCommands
from geometry_msgs.msg import PoseStamped
from status.msg import temp
from blinky.msg import RGB
from blinky.srv import BlinkyService

def reset_controls_speed():
    vel_vars.yaw_velocity = 0
    vel_vars.x_velocity = 0
    vel_vars.y_velocity = 0


def lbl_bg_red(thing):
    """sets a style sheet to the @param thing resulting in a red background"""
    thing.setStyleSheet('background-color:#ff0000')


class CentralUi(QtGui.QMainWindow):
    ##Qt signal for battery empty alarm
    empty_battery1_signal = QtCore.pyqtSignal()
    empty_battery2_signal = QtCore.pyqtSignal()
    high_ambiant_temp_signal = QtCore.pyqtSignal()

    high_core0_temp_signal = QtCore.pyqtSignal()
    high_core1_temp_signal = QtCore.pyqtSignal()
    high_core2_temp_signal = QtCore.pyqtSignal()
    high_core3_temp_signal = QtCore.pyqtSignal()
    high_ssd_temp_signal = QtCore.pyqtSignal()

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

        #self.style_data = ''
        #style = open('/home/david/robosub/catkin_ws/src/front_end/scripts/resource/darkorange.stylesheet', 'r')
        #self.style_data = style.read()
        #style.close()
        #self.ui.centralwidget.setStyleSheet(self.style_data)

        self.resize_sliders()
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

        ## creates the timers to enable or disable the ps3 controller for the controls systems
        self.ps3_timer_with_controls = QtCore.QTimer()
        ## creates the timers to enable or disable the keyboard controllers
        self.key_timer = QtCore.QTimer()
        self.ps3_timer_thrusters = QtCore.QTimer()
        self.thrust_pub_timer = QtCore.QTimer()
        ## create the ps3 controller object
        self.ps3 = PS3Controller_central.PS3Controller()

        ## place holder variable for internal battery status
        self.battery_empty = False
        
        # initiate pygame for the battery alarm
        pygame.init()
        pygame.mixer.init()

        # buttons connects
        QtCore.QObject.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        QtCore.QObject.connect(self.ui.attemptPS3, QtCore.SIGNAL("clicked()"), self.set_controller_timer)

        QtCore.QObject.connect(self.ui.blinky_red, QtCore.SIGNAL("clicked()"), lambda red = 255, blue = 0, green = 0 : self.send_color_blinky(red,green, blue))
        QtCore.QObject.connect(self.ui.blinky_green, QtCore.SIGNAL("clicked()"), lambda red = 0, blue = 0, green = 255 : self.send_color_blinky(red,green, blue))
        QtCore.QObject.connect(self.ui.blinky_blue, QtCore.SIGNAL("clicked()"), lambda red = 0, blue = 255, green = 0 : self.send_color_blinky(red,green, blue))
        QtCore.QObject.connect(self.ui.blinky_custom, QtCore.SIGNAL("clicked()"), self.custom_color)

        # critical state label changers
        self.empty_battery1_signal.connect(lambda lbl = self.ui.bat1_lbl: lbl_bg_red(lbl))
        self.empty_battery2_signal.connect(lambda lbl = self.ui.bat2_lbl: lbl_bg_red(lbl))
        self.high_ambiant_temp_signal.connect(lambda lbl = self.ui.ambiant_temp_lbl: lbl_bg_red(lbl))
        self.high_core0_temp_signal.connect(lambda lbl = self.ui.temp_core1: lbl_bg_red(lbl))
        self.high_core1_temp_signal.connect(lambda lbl = self.ui.temp_core2: lbl_bg_red(lbl))
        self.high_core2_temp_signal.connect(lambda lbl = self.ui.temp_core3: lbl_bg_red(lbl))
        self.high_core3_temp_signal.connect(lambda lbl = self.ui.temp_core4: lbl_bg_red(lbl))
        self.high_ssd_temp_signal.connect(lambda lbl = self.ui.hdd_temp_lbl: lbl_bg_red(lbl))

        # controller timer connect
        QtCore.QObject.connect(self.ps3_timer_with_controls, QtCore.SIGNAL("timeout()"), self.controller_update_controls)
        QtCore.QObject.connect(self.ps3_timer_thrusters, QtCore.SIGNAL("timeout()"), self.controller_update_thrusters)
        QtCore.QObject.connect(self.thrust_pub_timer, QtCore.SIGNAL("timeout()"), self.publish_thrusters)
        QtCore.QObject.connect(self.key_timer, QtCore.SIGNAL("timeout()"), self.keyboard_update)

        ## A timer to redraw the GUI (video monitors)
        self.redraw_timer = QtCore.QTimer(self)
        self.redraw_timer.timeout.connect(self.redraw_video_callback)
        self.redraw_timer.start(misc_vars.GUI_UPDATE_PERIOD)

        # change tab connect
        QtCore.QObject.connect(self.ui.tabWidget, QtCore.SIGNAL("currentChanged(int)"), self.change_tab)

        #thruster sliders connects
        QtCore.QObject.connect(self.ui.x_force, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.x_bal, QtCore.SIGNAL("valueChanged(int)"), self.x_force)
        QtCore.QObject.connect(self.ui.y_force, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.y_bal, QtCore.SIGNAL("valueChanged(int)"), self.y_force)
        QtCore.QObject.connect(self.ui.z_force, QtCore.SIGNAL("valueChanged(int)"), self.z_force)
        QtCore.QObject.connect(self.ui.z_bal, QtCore.SIGNAL("valueChanged(int)"), self.z_force)


    def custom_color(self):
        col = QtGui.QColorDialog.getColor()

        if col.isValid():
            hexa = col.name()
            hexa = hexa.lstrip('#')
            tup = tuple(int(hexa[i:i+2],16) for i in range(0,6,2))
            self.send_color_blinky(tup[0],tup[1], tup[2])

    ##initiallize the ros subscribers
    #
    #initiallize the ros node and starts all the ros subscribers and maps each topic to the correct callback
    #
    #note that all the topic names are set in the file "VARIABLES.py"
    #@param self the object pointer
    def start_ros_subscriber(self):
        rospy.init_node('Front_End', anonymous=True)
        rospy.Subscriber("/electrical_interface/depth", Int16, self.depth_callback)
        rospy.Subscriber("/electrical_interface/batteryVoltage1", Float32, self.bat_1)
        rospy.Subscriber("/electrical_interface/batteryVoltage2", Float32, self.bat_2)
        rospy.Subscriber("/camera_front_left/camera/image_rect_color", Image, self.front_left_pre_callback)
        rospy.Subscriber("/front_right_camera/image_rect_color", Image, self.front_right_pre_callback)
        rospy.Subscriber("/camera_down/camera/image_rect_color", Image, self.down_pre_callback)
        rospy.Subscriber("/front_cv/camera1", Image, self.front_post_left_callback)
        rospy.Subscriber("/front_cv/camera2", Image, self.front_post_right_callback)
        rospy.Subscriber("/down_cv/camera1", Image, self.down_post_callback)
        rospy.Subscriber('front_cv/data', VisibleObjectData, self.front_cv_data_callback)
        rospy.Subscriber('down_cv/data', VisibleObjectData, self.down_cv_data_callback)
        rospy.Subscriber('planner/task', String, self.planner_callback)
        rospy.Subscriber("/electrical_interface/pressure", Int16, self.pressure_callback)
        rospy.Subscriber("/electrical_interface/temperature", Int16, self.temp_callback)
        rospy.Subscriber("/status/tempearture", temp, self.cpu_hdd_temp_callback)
        #subscriber and callback for the 3d viz of pose data
        self.pose_ui.subscribe_topic('/state_estimation/pose')

    def cpu_hdd_temp_callback(self, temp):
        self.ui.temp_core1.setText(str(temp.core_0))
        if temp.core_0 > misc_vars.CPU_max_temp:
            self.high_core0_temp_signal.emit()
        self.ui.temp_core2.setText(str(temp.core_1))
        if temp.core_1 > misc_vars.CPU_max_temp:
            self.high_core1_temp_signal.emit()
        self.ui.temp_core3.setText(str(temp.core_2))
        if temp.core_2 > misc_vars.CPU_max_temp:
            self.high_core2_temp_signal.emit()
        self.ui.temp_core4.setText(str(temp.core_3))
        if temp.core_3 > misc_vars.CPU_max_temp:
            self.high_core3_temp_signal.emit()

        self.ui.hdd_temp_lbl.setText(str(temp.ssd))
        if temp.ssd > misc_vars.SSD_max_temp:
            self.high_ssd_temp_signal.emit()

    def send_color_blinky(self, red, green, blue):
        #build message
        color = RGB(red, green, blue)
        try:
            self.log_info("Waiting for service")
            rospy.wait_for_service('Blinky', timeout = 2)
            blinky = rospy.ServiceProxy('Blinky', BlinkyService)
            self.log_info("Making service call")
            res = blinky(color, 1)

            if res.success != 0:
                self.log_warning("Blinky request unsuccessful: %s"%res)

        except Exception as e:
            self.log_error("Exception: %s"%e)

    def change_tab(self, tab_no):
        if tab_no == 0:  # change to first tab
            reset_controls_speed()
            self.ps3_timer_thrusters.stop()
            self.log_debug("Stopping thruster publisher")
            self.thrust_pub_timer.stop()
            self.reset_thrusters()
            self.publish_thrusters()

        elif tab_no == 1:  # change to tab 2
            self.log_debug("Stopping publishing to /setPoints")
            self.ui.autonomousControl.setChecked(True)  # check autonomous controls
            self.set_controller_timer()  # send all inactive to controls, stop acquisition of key/ps3 input for co
            if self.ps3.controller_isPresent:
                self.ui.ps3_present.setChecked(True)
                self.log_debug("Re-tasking ps3 controller")
                self.ps3_timer_thrusters.start(misc_vars.controller_updateFrequency)
            self.thrust_pub_timer.start(200)
            self.log_debug("Starting thruster command publisher")
            self.check_imu_vals()

    def check_imu_vals(self):
        rospy.Subscriber("/electrical_interface/pose", PoseStamped, self.update_imu)

    def update_imu(self, pose):
        self.ui.imu_x.setText(str(pose.pose.orientation.x))
        self.ui.imu_y.setText(str(pose.pose.orientation.y))
        self.ui.imu_z.setText(str(pose.pose.orientation.z))
        self.ui.imu_w.setText(str(pose.pose.orientation.w))

    def controller_update_thrusters(self):
        self.ps3.updateController_for_thrusters()
        self.ui.fiel_thruste_1.setValue(self.ps3.fiel_thruste_1)
        self.ui.fiel_thruster_2.setValue(self.ps3.fiel_thruster_2)
        self.ui.fiel_thruster_3.setValue(self.ps3.fiel_thruster_3)
        self.ui.fiel_thruster_4.setValue(self.ps3.fiel_thruster_4)
        self.ui.fiel_thruster_5.setValue(self.ps3.fiel_thruster_5)
        self.ui.fiel_thruster_6.setValue(self.ps3.fiel_thruster_6)
        self.ui.thruster_stop.setChecked(self.ps3.thruster_stop)

    def reset_thrusters(self):
        self.ps3.updateController_for_thrusters()
        self.ui.fiel_thruste_1.setValue(0)
        self.ui.fiel_thruster_2.setValue(0)
        self.ui.fiel_thruster_3.setValue(0)
        self.ui.fiel_thruster_4.setValue(0)
        self.ui.fiel_thruster_5.setValue(0)
        self.ui.fiel_thruster_6.setValue(0)

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

    def pressure_callback(self, data):
        self.ui.pressure_lbl.setText(str(data.data))

    ##resize the sliders to fit the correct range of values
    def resize_sliders(self):
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
        self.ps3_timer_thrusters.stop()
        # radio button PS3
        if self.ui.manualControl.isChecked():
            self.log_debug("Starting ps3 control")
            self.keyboard_control = False
            self.key_timer.stop()

            # checks if the ps3 controller is present before starting the acquisition
            if self.ps3.controller_isPresent:
                self.ps3_timer_with_controls.start(misc_vars.controller_updateFrequency)
            else:
                self.log_info("PS3 Controller not found")

        # radio button KEYBOARD
        elif self.ui.keyboardControl.isChecked():
            self.log_debug("Starting keyboard control")
            self.ps3_timer_with_controls.stop()
            self.keyboard_control = True
            self.key_timer.start(misc_vars.controller_updateFrequency)
        # radio button AUTONOMOUS
        elif self.ui.autonomousControl.isChecked():
            self.log_debug("Stopping all controllers")
            self.keyboard_control = False
            self.ps3_timer_with_controls.stop()
            self.key_timer.stop()
            velocity_publisher.velocity_publisher(vel_vars.y_velocity, "/setPoints", 0)

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
        velocity_publisher.velocity_publisher(vel_vars.y_velocity,  "/setPoints", 1)

    ## Method for the ps3 control
    #
    # activated when the ps3_timer_with_controls times out
    #
    # updates the value of the ps3 controller data
    #
    # updates the ui with the values of the ps3 controller data
    #
    # publishes to the correct topic
    #  @param self the object pointer
    def controller_update_controls(self):
        # update the state of the controller
        self.ps3.updateController_for_controls_systems()

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
        velocity_publisher.velocity_publisher(-vel_vars.y_velocity, "/setPoints", 1)

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
        self.ui.ambiant_temp_lbl.setText(str(temp.data))
        if temp.data > misc_vars.max_temp:
            self.high_ambiant_temp_signal.emit()

    ##
    #appends the last message recieved from planner to a textbox in screen
    #@param self the object pointer
    #@param string_data data passed by the ros callback
    def planner_callback(self, string_data):
        self.ui.logObject.append(string_data.data)

    def log_debug(self, string_data):
        self.ui.logObject.append("[DEBUG] "+string_data)

        string_data = "[Front-end] "+string_data
        rospy.logdebug(string_data)

    def log_info(self, string_data):
        self.ui.logObject.append("[INFO] "+string_data)

        string_data = "[Front-end] "+string_data
        rospy.loginfo(string_data)

    def log_error(self, string_data):
        self.ui.logObject.append("[ERROR] "+string_data)

        string_data = "[Front-end] "+string_data
        rospy.logerr(string_data)

    def log_warning(self, string_data):
        self.ui.logObject.append("[WARN] "+string_data)

        string_data = "[Front-end] "+string_data
        rospy.logwarn(string_data)

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
        self.ui.bat1_lbl.setText(str(voltage_data.data))
        if voltage_data.data<misc_vars.low_battery_threshold:
            self.empty_battery1_signal.emit()

    def bat_2(self, voltage_data):
        self.ui.bat2_lbl.setText(str(voltage_data.data))
        if voltage_data.data<misc_vars.low_battery_threshold:
            self.empty_battery2_signal.emit()

    def check_low_bat(self):
        if self.ui.bat1_lbl.value() < misc_vars.low_battery_threshold or self.ui.bat2_lbl.value()<misc_vars.low_battery_threshold:
            self.open_low_battery_dialog()

    ## launch the popup for the low battery
    #
    #@param self the object pointer
    def open_low_battery_dialog(self):
        ## creates battery depleted ui
        warning_ui = BatteryWarningUi(self)
        warning_ui.exec_()

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
            self.ui.fiel_thruster_3.setValue(500)
        elif (self.ui.y_force.value() - self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.ui.fiel_thruster_3.setValue(-500)
        else:
            self.ui.fiel_thruster_3.setValue(self.ui.y_force.value() - self.ui.y_force.value() * self.ui.y_bal.value() / 100)

        if (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100) > 500:
            self.ui.fiel_thruster_4.setValue(-500)
        elif (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100) < -500:
            self.ui.fiel_thruster_4.setValue(500)
        else:
            self.ui.fiel_thruster_4.setValue(- (self.ui.y_force.value() + self.ui.y_force.value() * self.ui.y_bal.value() / 100))

    def z_force(self, data):
        if (self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.ui.fiel_thruster_5.setValue(500)
        elif (self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.ui.fiel_thruster_5.setValue(-500)
        else:
            self.ui.fiel_thruster_5.setValue(self.ui.z_force.value() - self.ui.z_force.value() * self.ui.z_bal.value() / 100)

        if (self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100) > 500:
            self.ui.fiel_thruster_6.setValue(500)
        elif (self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100) < -500:
            self.ui.fiel_thruster_6.setValue(-500)
        else:
            self.ui.fiel_thruster_6.setValue(self.ui.z_force.value() + self.ui.z_force.value() * self.ui.z_bal.value() / 100)


def sigint_handler(*args):
    """Handler for the SIGINT signal."""
    sys.stderr.write('\r')
    #if QtGui.QMessageBox.question(None, '', "Are you sure you want to quit?",
    #                              QtGui.QMessageBox.Yes | QtGui.QMessageBox.No,
    #                              QtGui.QMessageBox.Yes) == QtGui.QMessageBox.Yes:
    rospy.loginfo("[Front-end] EXITING")
    QtGui.QApplication.quit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, sigint_handler)
    app = QtGui.QApplication(sys.argv)
    timer = QtCore.QTimer()
    timer.start(500)  # You may change this if you wish.
    timer.timeout.connect(lambda: None)  # Let the interpreter run each 500 ms.
    AppWindow = CentralUi()
    AppWindow.show()
    sys.exit(app.exec_())
