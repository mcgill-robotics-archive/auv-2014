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
#increments the z position (robot lowers) in VARIABLES by a step defined in the same file
#@param self th eobject pointer
def z_lower():
    vel_vars.z_position += vel_vars.z_position_step


##
#decrements the z position (robot rises) in VARIABLES by a step defined in the same file
#@param self th eobject pointer
def z_rise():
    if vel_vars.z_position > 0:
        vel_vars.z_position -= vel_vars.z_position_step


##
#Sets z position in VARIABLES to 0, robot surfaces
#@param self th eobject pointer
def z_surface():
    vel_vars.z_position = 0


##
#  Class containing all relevant commands to fetch data from a ps3 controller
class PS3Controller(object):
    ##
    #Constructor of the ps3Controller. 
    #It basically initialize the joystick so we can fetch data form it.
    def __init__(self):
        pygame.init()
        pygame.joystick.init()

        ##Default config as ps3 controller not existant
        #will be changed to true if the controller is initialized. 
        self.controller_isPresent=False
        self.thruster_stop = False
        self.fiel_thruste_1 = 0
        self.fiel_thruster_2 = 0
        self.fiel_thruster_3 = 0
        self.fiel_thruster_4 = 0
        self.fiel_thruster_5 = 0
        self.fiel_thruster_6 = 0
        
        if pygame.joystick.get_count() == 0:
            self.controller_isPresent = False
        elif pygame.joystick.get_count() == 1:
            self.controller_isPresent = True

            ##The initialized controller
            self.controller = pygame.joystick.Joystick(0)
            self.controller.init()
        else:
            self.controller_isPresent = False

    ##
    #It reads the data from the controller
    # (the queue of event more precisely) and updates the Global data of the instance.
    #@param self the object pointer
    def updateController_for_controls_systems(self):
        """This method fetches the information form the ps3 controller parses the data and returns values ready to be
        published for the implementation with the controls systems, i.e. it should be used with the 'Sensor Information'
        tab. """
        # If a changed occurred, the value will be updated; else the value will be the last one fetched.

        if self.controller_isPresent:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:

                    if self.controller.get_button(7):  # left arrow
                        z_surface()
                    elif self.controller.get_button(6):  # down arrow
                        z_lower()
                    #elif self.controller.get_button(5):  # right arrow
                    elif self.controller.get_button(4):  # up arrow
                        z_rise()

                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    vel_vars.y_velocity = -vel_vars.MAX_LINEAR_VEL*self.controller.get_axis(0)  # left left/right axis
                    vel_vars.x_velocity = -vel_vars.MAX_LINEAR_VEL*self.controller.get_axis(1)  # left front/back axis
                    vel_vars.yaw_velocity = -vel_vars.MAX_YAW_VEL*self.controller.get_axis(2)  # right left/right axis


    def updateController_for_thrusters(self):
        """Fetches the values of the ps3 controller axies and parses them to be published as voltage
         command for the arduino node directly.  """
        # If a changed occurred, the value will be updated; else the value will be the last one fetched.
        if self.controller_isPresent:
            for anEvent in pygame.event.get():
                if anEvent.type == pygame.locals.JOYBUTTONDOWN:

                    if self.controller.get_button(3):  # left arrow
                        if self.thruster_stop:
                            self.thruster_stop = False
                        else:
                            self.thruster_stop = True

                elif anEvent.type == pygame.locals.JOYAXISMOTION:
                    if self.controller.get_button(10):  # find l1
                        self.fiel_thruste_1 = ( -500*self.controller.get_axis(1))
                        self.fiel_thruster_2 = ( - 500*self.controller.get_axis(1))
                        # ui.fiel_thruste_1 & x2 are the same, left front/back

                        if self.controller.get_axis(2) == 0:  # if not turning yaw (right x)
                            self.fiel_thruster_3 = (500*self.controller.get_axis(0))
                            self.fiel_thruster_4 = (-500*self.controller.get_axis(0))
                    if self.controller.get_button(11):  # find r1
                        self.fiel_thruster_5 = (-500*self.controller.get_axis(3))
                        self.fiel_thruster_6 = (-500*self.controller.get_axis(3))

                        self.fiel_thruster_3 = (500*self.controller.get_axis(2))
                        self.fiel_thruster_4 = (500*self.controller.get_axis(2))

