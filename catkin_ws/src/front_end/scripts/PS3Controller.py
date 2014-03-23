## @package PS3Controller
#
#  Main file for PS3 controller
#
#  @author David Lavoie-Boutin

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

##
#  Class containing all relevant commands to fetch data from a ps3 controller
class PS3Controller(object):
    ##
    #Constructor of the ps3Controller. 
    #It basically initialize the joystick so we can fetch data form it.
    def __init__(self):
        print "[INFO] PS3Controller::__init__"
        pygame.init()
        pygame.joystick.init()

        ##Default config as ps3 controller not existant
        #will be changed to true if the controller is initialized. 
        self.controller_isPresent=False
        
        if pygame.joystick.get_count() == 0:
            print "The controller is not connected"
            print "Shutting down the process..."
            self.controller_isPresent = False
        elif pygame.joystick.get_count() == 1:
            print "One controller is connected"
            print "All set to go"
            self.controller_isPresent = True
            self.initialize_controller()
        else:
            print "There is more than one controller connected. Is this correct?"
            print "Shutting down the process..."
            self.controller_isPresent = False

    ##
    # This part will set all the Global variables used in PS3Controller.
    # It is only called if the constructor finds only 1 joystick
    # @param self the object pointer 
    def initialize_controller(self):
        ##The initialized controller
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        print "The initialized Joystick is: " + self.controller.get_name()

    ##
    #increments the z position (robot lowers) in VARIABLES by a step defined in the same file
    #@param self th eobject pointer
    def z_lower(self):
        vel_vars.z_position += vel_vars.z_position_step

    ##
    #decrements the z position (robot rises) in VARIABLES by a step defined in the same file
    #@param self th eobject pointer
    def z_rise(self):
        if vel_vars.z_position > 0:
            vel_vars.z_position -= vel_vars.z_position_step
    
    ##
    #Sets z position in VARIABLES to 0, robot surfaces
    #@param self th eobject pointer
    def z_surface(self):
        vel_vars.z_position = 0

    ##
    #It reads the data from the controller (the queue of event more precisely) and updates the Global data of the instance.
    #@param self the object pointer
    def updateController(self):
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
                    #vel_vars.pitch_velocity = -vel_vars.MAX_PITCH_ANGLE*self.controller.get_axis(3)  # right front/back axis