#!/usr/bin/env python
import roslib; roslib.load_manifest('blinky')
import rospy

from std_msgs.msg import Float32
from blinky.msg import *
from blinky.srv import *

CYAN = RGB(0,255,255)
ORANGE = RGB(255,105,0)
RED = RGB(255,0,0)

# Minimum and maximum battery voltage values in volts
# Ask power section for accurate values
MIN_VOLTAGE = 21.0
MAX_VOLTAGE = 24.0
LOW_BATTERY = False

ledCount = 30

# Handles the ROS-specific communication with the service
# colors: array of RGB
def Battery1_sendColors(colors):
    try:
        # wait for the update_battery1_lights service
        rospy.wait_for_service('update_battery1_lights')

        # get access to the UpdateBattery1Lights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('update_battery1_lights', UpdateBattery1Lights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "UpdateBattery1Lights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e


def Battery2_sendColors(colors):
    try:
        # wait for the update_battery2_lights service
        rospy.wait_for_service('update_battery2_lights')

        # get access to the UpdateBattery2Lights service from the blinky server
        blinky_proxy = rospy.ServiceProxy('update_battery2_lights', UpdateBattery2Lights)

        # call service
        res = blinky_proxy(colors)

        if res.success != 0:
            print "UpdateBattery2Lights request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

# Recieves a voltage message and displays in on the blinky tape.
# Units are in volts, in floating point.
def process_voltage1(voltage):
    global LOW_BATTERY

    volts = voltage.data

    charge_level = (int)(((volts - MIN_VOLTAGE)/(MAX_VOLTAGE - MIN_VOLTAGE))*15)
    colors = []

    if volts <= MIN_VOLTAGE and volts >= 10.0:
        LOW_BATTERY = True
    else:
        LOW_BATTERY = False

    if charge_level < 0:
        charge_level = 0
    elif charge_level > 15:
        charge_level = 15

    for i in range(15):
        if charge_level > i:
            colors.append(CYAN)
        else:
            colors.append(ORANGE)

    Battery1_sendColors(colors)


def process_voltage2(voltage):
    global LOW_BATTERY

    volts = voltage.data

    charge_level = (int)(((volts - MIN_VOLTAGE)/(MAX_VOLTAGE - MIN_VOLTAGE))*15)
    colors = []

    if charge_level < 0:
        LOW_BATTERY = True
        charge_level = 0
    elif charge_level > 15:
        LOW_BATTERY = False
        charge_level = 15

    for i in range(15):
        if charge_level > i:
            colors.append(CYAN)
        else:
            colors.append(ORANGE)

    Battery2_sendColors(colors)


def warn_low_battery(state):
    try:
        rospy.wait_for_service('warning_lights')
        blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
        result = blinky_proxy(RED, 0.5, state)

        if result.success != 0:
            print 'WarningUpdateLights request unsuccessful: %s' % (result)

    except rospy.exceptions.ROSInterruptException:
        pass


def BatteryListener():
    rospy.init_node('battery_lights')
    rospy.Subscriber("electrical_interface/batteryVoltage1", Float32, process_voltage1)
    rospy.Subscriber("electrical_interface/batteryVoltage2", Float32, process_voltage2)


if __name__ == "__main__":
    BatteryListener()
    if not rospy.is_shutdown():
        warn_low_battery(LOW_BATTERY)
