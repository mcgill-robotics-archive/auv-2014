#!usr/bin/python
'''
Created on Aug 10th 2013

@author: Jean-Sebastien Dery
'''

import subprocess
import os
import sys
import time

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
        if (numberOfDeviceConnected == 0):
            print "The controller is not connected to the Raspberry Pi"
            print "Shutting down the process..."
            sys.exit()
        elif (numberOfDeviceConnected == 1):
            print "One controller is connected to the Raspberry Pi"
            self.initGlobalVariables()
        else:
            print "There is more than one controller connected to the Raspberry Pi. Is this correct?"
            print "Shutting down the process..."
            sys.exit()

    def initGlobalVariables(self):
        """
		This method will set all the Global variables used in PS3Controller.
		"""
        print self.className + "initGlobalVariables()"
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print "The initialized Joystick is: %s" + self.controller.get_name()
        self.rightJoystickX = 0
        self.rightJoystickY = 0
        self.yawAngle = 90
        self.pitchAngle = 120
        self.axisDeltaConstant = 2
        self.maxYawAngle = 180
        self.minYawAngle = 0
        self.maxPitchAngle = 160
        self.minPitchAngle = 95
        self.joystickAxisTreshold = 0.9
        self.isAutonomous = False
        self.isShooting = False

    def updateController(self):
        """
		It reads the data from the controller (the queue of event more precisely) and updates the Global data of the instance.
		"""

        # If a changed occurred, the value will be updated; else the value will be the last one fetched.
        for anEvent in pygame.event.get():
            # This is for the Manual or Autonomous control.
            if (anEvent.type == pygame.locals.JOYBUTTONDOWN and self.controller.get_button(15)):
                self.__invertIsAutonomous()
            # This is for the Shooting control.
            if (anEvent.type == pygame.locals.JOYBUTTONDOWN and self.controller.get_button(14)):
                self.__invertCanShoot()
            if (self.isAutonomous == False):
                # This is for the ServoMotor angles.
                if (anEvent.type == pygame.locals.JOYAXISMOTION):
                    self.rightJoystickX = self.controller.get_axis(2)
                    self.rightJoystickY = self.controller.get_axis(3)

        if (self.rightJoystickX > self.joystickAxisTreshold):
            self.__diminishYawAngle()
        elif (self.rightJoystickX < -self.joystickAxisTreshold):
            self.__augmentYawAngle()

        if (self.rightJoystickY > self.joystickAxisTreshold):
            self.__augmentPitchAngle()
        elif (self.rightJoystickY < -self.joystickAxisTreshold):
            self.__diminishPitchAngle()

    def __invertIsAutonomous(self):
        """
		Inverts the state control state of the robot.
		"""
        if (self.isAutonomous == False):
            self.isAutonomous = True
        else:
            self.isAutonomous = False

    def __invertCanShoot(self):
        """
		Inverts the shooting state of the robot.
		"""
        if (self.isShooting == False):
            self.isShooting = True
        else:
            self.isShooting = False

    def isRobotAutonomous(self):
        """
		Returns the control state of the robot.
		"""
        return (self.isAutonomous)

    def canShoot(self):
        """
		Returns the shooting state of the robot.
		"""
        return (self.isShooting)

    def getRightStickData(self):
        """
		Returns a tuple of the yawAngle and pitchAngle of rotation values.
		"""
        print self.className + "getRightStickData()"
        return (self.yawAngle, self.pitchAngle)

    def __augmentYawAngle(self):
        """
		Augments the yawAngle by the axisDeltaConstant up until the maxYawAngle.
		"""
        # Augments the value.
        if (self.yawAngle < self.maxYawAngle):
            self.yawAngle = self.yawAngle + self.axisDeltaConstant
        # Sets the value to the maximum if it went higher.
        if (self.yawAngle > self.maxYawAngle):
            self.yawAngle = self.maxYawAngle

    def __diminishYawAngle(self):
        """
		Diminishes the yawAngle by the axisDeltaConstant down until the minYawAngle.
		"""
        # Augments the value.
        if (self.yawAngle > self.minYawAngle):
            self.yawAngle = self.yawAngle - self.axisDeltaConstant
        # Sets the value to the maximum if it went higher.
        if (self.yawAngle < self.minYawAngle):
            self.yawAngle = self.minYawAngle

    def __augmentPitchAngle(self):
        # Augments the value.
        if (self.pitchAngle < self.maxPitchAngle):
            self.pitchAngle = self.pitchAngle + self.axisDeltaConstant
        # Sets the value to the maximum if it went higher.
        if (self.pitchAngle > self.maxPitchAngle):
            self.pitchAngle = self.maxPitchAngle

    def __diminishPitchAngle(self):
        # Augments the value.
        if (self.pitchAngle > self.minPitchAngle):
            self.pitchAngle = self.pitchAngle - self.axisDeltaConstant
        # Sets the value to the maximum if it went higher.
        if (self.pitchAngle < self.minPitchAngle):
            self.pitchAngle = self.minPitchAngle