#!usr/bin/python
"""
Created on Aug 10th 2013

__author__: Jean-Sebastien Dery
 revision by David Lavoie-Boutin
"""

import os
import sys

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

        self.yaw_speed = 0
        self.pitch_speed = 0
        self.horizontal_side_speed = 0
        self.horizontal_front_speed = 0

        self.square = 0
        self.triangle = 0
        self.o = 0
        self.x = 0

        self.z_position = 0.0 # position in meters
        self.z_position_step = 0.5

        self.l1 = 0
        self.l2 = 0
        self.l3 = 0
        self.r1 = 0
        self.r2 = 0
        self.r3 = 0
        self.start = 0
        self.select = 0

    def initialize_controller(self):
        #This part will set all the Global variables used in PS3Controller.
        #It is only called if the constructor finds only 1 joystick
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.controller_name =  self.controller.get_name()
        print "The initialized Joystick is: " + self.controller_name

    def z_lower(self):
        self.z_position += self.z_position_step

    def z_rise(self):
        if self.z_position>0:
            self.z_position -= self.z_position_step

    def z_surface(self):
        self.z_position = 0

    def updateController(self):
        """
        It reads the data from the controller (the queue of event more precisely) and updates the Global data of the instance.
        """

        # If a changed occurred, the value will be updated; else the value will be the last one fetched.
        if self.controller_isPresent:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:
                    #if self.controller.get_button(15):
                    #elif self.controller.get_button(14):
                    #elif self.controller.get_button(13):
                    #elif self.controller.get_button(12):
                    #elif self.controller.get_button(11):
                    #elif self.controller.get_button(10):
                    #elif self.controller.get_button(9):
                    #elif self.controller.get_button(8):

                    if self.controller.get_button(7):  # left arrow
                        self.z_surface()
                    elif self.controller.get_button(6):  # down arrow
                        self.z_lower()
                    #elif self.controller.get_button(5):  # right arrow
                    elif self.controller.get_button(4):  # up arrow
                        self.z_rise()

                    #elif self.controller.get_button(3):
                    #elif self.controller.get_button(2):
                    #elif self.controller.get_button(1):
                    #elif self.controller.get_button(0):
                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    self.horizontal_side_speed = -0.2*self.controller.get_axis(0)  # left left/right axis
                    self.horizontal_front_speed = -0.2*self.controller.get_axis(1)  # left front/back axis
                    self.yaw_speed = -0.2*self.controller.get_axis(2)  # right left/right axis
                    self.pitch_speed = -0.2*self.controller.get_axis(3)  # right front/back axis

    def returnButtons(self):
        return self.select, self.l3, self.r3, self.start, self.l2, self.r2, self.l1, self.r1, self.triangle, self.o, self.x, self.square