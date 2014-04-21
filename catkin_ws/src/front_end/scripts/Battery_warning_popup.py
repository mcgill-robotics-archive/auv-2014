__author__ = 'David'

from low_battery_warning import*
from PyQt4 import QtCore, QtGui
from VARIABLES import *  # file containing all the shared variables and parameters
import pygame

alarm_file = ""


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


def stop_alarm():
    pygame.mixer.music.stop()


## start the alarm sound
#
#@param self the object pointer
def play_alarm():
    # initiate pygame for the battery alarm
    pygame.init()
    pygame.mixer.init()
    pygame.mixer.music.load(alarm_file)
    pygame.mixer.music.play(-1, 0)

    while pygame.mixer.music.get_busy():
        pygame.time.Clock().tick(10)