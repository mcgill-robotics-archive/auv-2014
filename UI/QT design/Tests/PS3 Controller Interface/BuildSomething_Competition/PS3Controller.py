#!usr/bin/python
"""
Created on Aug 10th 2013

@author: Jean-Sebastien Dery
adaptation by David Lavoie-Boutin
"""

import os
import sys
from pygame import locals
import pygame

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
            sys.exit()
        elif numberOfDeviceConnected == 1:
            print "One controller is connected"
            print "All set to go"
            self.initGlobalVariables()
        else:
            print "There is more than one controller connected. Is this correct?"
            print "Shutting down the process..."
            sys.exit()

    def initGlobalVariables(self):
        """
        This method will set all the Global variables used in PS3Controller.
        """
        print self.className + "initGlobalVariables()"
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print "The initialized Joystick is: " + self.controller.get_name()
        self.rJoyX = 0
        self.rJoyY = 0
        self.lJoyX = 0
        self.lJoyY = 0

        self.square = 0
        self.triangle = 0
        self.o = 0
        self.x = 0

        self.up = 0
        self.left = 0
        self.down = 0
        self.right = 0

        self.l1 = 0
        self.l2 = 0
        self.l3 = 0
        self.r1 = 0
        self.r2 = 0
        self.r3 = 0
        self.start = 0
        self.select = 0
        self.ps = 0

    def inverseSquare(self):
        if self.square == 0:
            self.square = 1
        else:
            self.square = 0

    def inverseTriangle(self):
        if self.triangle == 0:
            self.triangle = 1
        else:
            self.triangle = 0

    def updateController(self):
        """
        It reads the data from the controller (the queue of event more precisely) and updates the Global data of the instance.
        """

        # If a changed occurred, the value will be updated; else the value will be the last one fetched.

        for anEvent in pygame.event.get():
            if anEvent.type == pygame.locals.JOYBUTTONDOWN:
                if self.controller.get_button(15):
                    self.inverseSquare()
                elif self.controller.get_button(14):
                    self.x = 1
                elif self.controller.get_button(13):
                    self.o = 1
                elif self.controller.get_button(12):
                    self.inverseTriangle()

                self.controller.get_button(11)
                self.controller.get_button(10)
                self.controller.get_button(9)
                self.controller.get_button(8)
                self.controller.get_button(7)
                self.controller.get_button(6)
                self.controller.get_button(5)
                self.controller.get_button(4)
                self.controller.get_button(3)
                self.controller.get_button(2)
                self.controller.get_button(1)
                self.controller.get_button(0)

            elif anEvent.type == pygame.locals.JOYAXISMOTION:
                self.lJoyX = self.controller.get_axis(0)
                self.lJoyY = self.controller.get_axis(1)
                self.rJoyX = self.controller.get_axis(2)
                self.rJoyY = self.controller.get_axis(3)

    def getRightStickData(self):
        """
        Returns a tuple of the yawAngle and pitchAngle of rotation values.
        """
        return self.rJoyX, self.rJoyY

