#!/usr/bin/env python

# IMPORTS
import sensors
import socket
import roslib
import rospy
from blinky.srv import *
from blinky.msg import *
from status.msg import *
from std_msgs.msg import *
roslib.load_manifest('status')

# VARIABLES
frequency = 0.00            # BLINKING FREQUENCY            Hz
temperatures = temp()       # MESSAGE TO PUBLISH
warning = False             # STORES WHETHER WARNING OR NOT

# CONSTANTS
COLOR = [RGB(255, 255, 0)]  # COLOR TO FLASH            YELLOW
HOST = ('127.0.0.1', 7634)  # HDDTEMP DAEMON SOCKET
CPU_THRESHOLD = 90          # THRESHOLD FOR CPU CORES        C
SSD_THRESHOLD = 65          # THRESHOLD FOR SSD              C

# SET UP NODE AND TOPIC
rospy.init_node('status')
temperature_topic = rospy.Publisher('temperature', temp)
rate = rospy.Rate(10)

# SET UP SENSORS
sensors.init()


def update():
    ''' Updates temperatures '''
    # GET CPU CORE TEMPERATURES
    for chip in sensors.iter_detected_chips():
        for feature in chip:
            name = feature.name
            if name.startswith('temp'):
                if name.endswith('2'):
                    temperatures.core_0 = int(feature.get_value())
                elif name.endswith('3'):
                    temperatures.core_1 = int(feature.get_value())
                elif name.endswith('4'):
                    temperatures.core_2 = int(feature.get_value())
                elif name.endswith('5'):
                    temperatures.core_3 = int(feature.get_value())

    # SET UP SOCKET
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(HOST)
    data = sock.recv(4096)
    sock.close()

    # GET SSD TEMPERATURE
    try:
        temperatures.ssd = int(data.split('|')[3])
    except IndexError:
        pass


def checkup():
    ''' Checks if temperatures are above thresholds '''
    # COMPARE TO THRESHOLDS
    ok = True
    if temperatures.core_0 > CPU_THRESHOLD:
        ok = False
    if temperatures.core_1 > CPU_THRESHOLD:
        ok = False
    if temperatures.core_2 > CPU_THRESHOLD:
        ok = False
    if temperatures.core_3 > CPU_THRESHOLD:
        ok = False
    if temperatures.ssd > SSD_THRESHOLD:
        ok = False

    # SET BLINKYTAPE ACCORDINGLY
    if not ok:
       blinky(True)
    else:
       blinky(False)


def publish():
    ''' Publishes temperatures '''
    rospy.loginfo(temperatures)
    temperature_topic.publish(temperatures)
    rate.sleep()


def blinky(state):
    ''' Lights up blinkytape for warnings '''
    global COLOR, frequency, warning

    # INCREASE FREQUENCY
    if state:
        frequency += 0.01
        warning = True
    else:
        frequency = 0.00

    # CALL SERVICE IF NEEDED
    if state or warning:
        try:
            rospy.wait_for_service('warning_lights')
            blinky_proxy = rospy.ServiceProxy('warning_lights', WarningLights)
            result = blinky_proxy(COLOR, frequency, state)

            if result.success != 0:
                print 'WarningUpdateLights request unsuccessful: %s' % (result)
        
        except rospy.exceptions.ROSInterruptException:
            pass

    # RESET FLAG
    if not state:
        warning = False

# MAIN
if __name__ == '__main__':
    while True:
        try:
            if not rospy.is_shutdown():
                update()
                publish()
                checkup()

            else:
                break

        except Exception as e:
            print e
            sensors.cleanup()
            blinky(False)
            break
