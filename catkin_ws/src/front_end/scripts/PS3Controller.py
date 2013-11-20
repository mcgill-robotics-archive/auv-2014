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
            sys.exit()
        elif numberOfDeviceConnected == 1:
            print "One controller is connected"
            print "All set to go"
        else:
            print "There is more than one controller connected. Is this correct?"
            print "Shutting down the process..."
            sys.exit()

        #This part will set all the Global variables used in PS3Controller.
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print "The initialized Joystick is: " + self.controller.get_name()
        self.yaw_speed = 0
        self.pitch_speed = 0
        self.horizontal_front_speed = 0
        self.horizontal_side_speed = 0

        self.square = 0
        self.triangle = 0
        self.o = 0
        self.x = 0

        self.vertical_speed = 0
        self.max_vertical_speed = 10

        self.l1 = 0
        self.l2 = 0
        self.l3 = 0
        self.r1 = 0
        self.r2 = 0
        self.r3 = 0
        self.start = 0
        self.select = 0

    def inverseSquare(self):
        if self.square == 0:
            self.square =1
        else:
            self.square = 0

    def inverseTriangle(self):
        if self.triangle == 0:
            self.triangle = 1
        else:
            self.triangle = 0

    def inverseO(self):
            if self.o == 0:
                self.o = 1
            else:
                self.o = 0

    def inverseX(self):
            if self.x == 0:
                self.x = 1
            else:
                self.x = 0

    def inverseR1(self):
        if self.r1 == 0:
            self.r1 = 1
        else:
            self.r1 = 0

    def inverseL1(self):
        if self.l1 == 0:
            self.l1 = 1
        else:
            self.l1 = 0

    def inverseR2(self):
        if self.r2 == 0:
            self.r2 = 1
        else:
            self.r2 = 0

    def inverseL2(self):
        if self.l2 == 0:
            self.l2 = 1
        else:
            self.l2 = 0

    def inverseR3(self):
        if self.r3 == 0:
            self.r3 = 1
        else:
            self.r3 = 0

    def inverseL3(self):
        if self.l3 == 0:
            self.l3 = 1
        else:
            self.l3 = 0

    def inverseStart(self):
        if self.start == 0:
            self.start = 1
        else:
            self.start = 0

    def inverseSelect(self):
        if self.select == 0:
            self.select = 1
        else:
            self.select = 0

    def increase_vertical_speed(self):
        if self.max_vertical_speed > self.vertical_speed:
            self.vertical_speed += 1

    def decrease_vertical_speed(self):
        if self.vertical_speed > -self.max_vertical_speed:
            self.vertical_speed -= 1

    def reset_vertical_speed(self):
        self.vertical_speed = 0

    def max_vertical_speed_up(self):
        self.vertical_speed = self.max_vertical_speed

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
                    self.inverseX()
                elif self.controller.get_button(13):
                    self.inverseO()
                elif self.controller.get_button(12):
                    self.inverseTriangle()
                elif self.controller.get_button(11):
                    self.inverseR1()
                elif self.controller.get_button(10):
                    self.inverseL1()
                elif self.controller.get_button(9):
                    self.inverseR2()
                elif self.controller.get_button(8):
                    self.inverseL2()

                elif self.controller.get_button(7):  # left arrow
                    self.reset_vertical_speed()
                elif self.controller.get_button(6):  # down arrow
                    self.decrease_vertical_speed()
                elif self.controller.get_button(5):  # right arrow
                    self.max_vertical_speed_up()
                elif self.controller.get_button(4):  # up arrow
                    self.increase_vertical_speed()

                elif self.controller.get_button(3):
                    self.inverseStart()
                elif self.controller.get_button(2):
                    self.inverseR3()
                elif self.controller.get_button(1):
                    self.inverseL3()
                elif self.controller.get_button(0):
                    self.inverseSelect()

#TODO : confirm correct assignment of the axis
            elif anEvent.type == pygame.locals.JOYAXISMOTION:
                self.horizontal_front_speed = self.controller.get_axis(0)  # left front/back axis
                self.horizontal_side_speed = self.controller.get_axis(1)  #left left/right axis
                self.yaw_speed = self.controller.get_axis(2)  # right left/right axis
                self.pitch_speed = self.controller.get_axis(3)  #

    def returnButtons(self):
        return self.select, self.l3, self.r3, self.start, self.l2, self.r2, self.l1, self.r1, self.triangle, self.o, self.x, self.square