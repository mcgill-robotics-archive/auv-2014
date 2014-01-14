#!usr/bin/python

## @package central_app
#  @author David Lavoie-Boutin
#
#  Main file for McGill Robotics AUV Design Team testing User Interface
"""
Created on Aug 10th 2013

__author__: Jean-Sebastien Dery
adaptation for McGill Robotics by David Lavoie-Boutin

Class containing all relevant commands to fetch data from a ps3 controller
"""

import os
import sys
from VARIABLES import vel_vars

try:
    from pygame import locals
    import pygame
except ImportError:
    print "Cannot import the PyGame library \nNow terminating the process..."
    sys.exit()

os.environ["SDL_VIDEODRIVER"] = "dummy"


class PS3Controller(object):
    def __init__(self):
        """
        Constructor of the ps3Controller. It basically initialize the joystick so we can fetch data form it.
        """
        self.className = "[INFO] PS3Controller::"
        print self.className + "__init__"
        pygame.init()
        pygame.joystick.init()
        numberOfDeviceConnected = pygame.joystick.get_count()

        if numberOfDeviceConnected == 0:
            print "The controller is not connected"
            print "Shutting down the process..."
            self.controller_isPresent = False
        elif numberOfDeviceConnected == 1:
            print "One controller is connected"
            print "All set to go"
            self.controller_isPresent = True
            self.initialize_controller()
        else:
            print "There is more than one controller connected. Is this correct?"
            print "Shutting down the process..."
            self.controller_isPresent = False

        self.controller = None
        self.controller_name = None

    def initialize_controller(self):
        """
        This part will set all the Global variables used in PS3Controller.
        It is only called if the constructor finds only 1 joystick
        """
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.controller_name =  self.controller.get_name()
        print "The initialized Joystick is: " + self.controller_name

    def z_lower(self):
        vel_vars.z_position += vel_vars.z_position_step

    def z_rise(self):
        if vel_vars.z_position > 0:
            vel_vars.z_position -= vel_vars.z_position_step

    def z_surface(self):
        vel_vars.z_position = 0

    def updateController(self):
        """
        It reads the data from the controller (the queue of event more precisely) and updates the Global data of the instance.
        """

        # If a changed occurred, the value will be updated; else the value will be the last one fetched.
        if self.controller_isPresent:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:

                    if self.controller.get_button(7):  # left arrow
                        self.z_surface()
                    elif self.controller.get_button(6):  # down arrow
                        self.z_lower()
                    #elif self.controller.get_button(5):  # right arrow
                    elif self.controller.get_button(4):  # up arrow
                        self.z_rise()

                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    vel_vars.x_velocity = -vel_vars.MAX_LINEAR_VEL*self.controller.get_axis(0)  # left left/right axis
                    vel_vars.y_velocity = -vel_vars.MAX_LINEAR_VEL*self.controller.get_axis(1)  # left front/back axis
                    vel_vars.yaw_velocity = -vel_vars.MAX_YAW_VEL*self.controller.get_axis(2)  # right left/right axis
                    vel_vars.pitch_velocity = -vel_vars.MAX_PITCH_VEL*self.controller.get_axis(3)  # right front/back axis