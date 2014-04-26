#!/usr/bin/env python

# IMPORTS
import sensors
import roslib
import rospy
import blinky
from blinky.srv import *
from blinky.msg import *
from status.msg import *
import socket
import time
import os
roslib.load_manifest('status')

# SET UP NODE AND TOPIC
rospy.init_node('status')
temperature = rospy.Publisher('temperature', temp)
rate = rospy.Rate(10)

# SET UP MESSAGE
temps = temp()

# SET UP HDDTEMP
HOST = '127.0.0.1'
PORT = 7634

# SET UP SENSORS
sensors.init()

# THRESHOLDS IN CELCIUS
CPU_THRESHOLD = 90
SSD_THRESHOLD = 65

# VARIABLES
FREQUENCY = 0
YELLOW = [RGB(255, 255, 0)]


def update():
    ''' Updates temperatures '''
    # GET CPU CORE TEMPERATURES
    for chip in sensors.iter_detected_chips():
        for feature in chip:
            name = feature.name
            if name.startswith('temp'):
                if name.endswith('2'):
                    temps.core_0 = int(feature.get_value())
                elif name.endswith('3'):
                    temps.core_1 = int(feature.get_value())
                elif name.endswith('4'):
                    temps.core_2 = int(feature.get_value())
                elif name.endswith('5'):
                    temps.core_3 = int(feature.get_value())

    # SET UP SOCKET
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    data = sock.recv(4096)
    sock.close()

    # GET SSD TEMPERATURE
    try:
        temps.ssd = int(data.split('|')[3])
    except IndexError:
        pass


def checkup():
    ''' Checks if temperatures are above thresholds '''
    # COMPARE
    ok = True
    if temps.core_0 > CPU_THRESHOLD:
        ok = False
    if temps.core_1 > CPU_THRESHOLD:
        ok = False
    if temps.core_2 > CPU_THRESHOLD:
        ok = False
    if temps.core_3 > CPU_THRESHOLD:
        ok = False
    if temps.ssd > SSD_THRESHOLD:
        ok = False

    # SET BLINKYTAPE ACCORDINGLY
    if not ok:
       FREQUENCY += 1
       blinky()
    else:
       FREQUENCY = 0
       blinky()


def publish():
    ''' Publishes temperatures '''
    rospy.loginfo(temps)
    temperature.publish(temps)
    rate.sleep()


def blinky():
    ''' Lights up blinkytape for warnings '''
    try:
        rospy.wait_for_service('blinky')
        Blinky = rospy.ServiceProxy('blinky', BlinkyService)
        result = Blinky(YELLOW, 1)

        if result.success != 0:
            print "Blinky request unsuccessful: %s" % result

    except Exception as e:
        print "\n%s: %s" % ('Exception', e)


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

        except Exception:
            sensors.cleanup()
            break
