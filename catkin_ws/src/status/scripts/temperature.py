#!/usr/bin/env python

# IMPORTS
import sensors
import roslib; roslib.load_manifest('status')
import rospy
import blinky
from blinky.msg import *
from blinky.srv import *
from status.msg import *
import socket
import time
import os

# SET UP NODES AND TOPICS
rospy.init_node('temperature')
temperature = rospy.Publisher('temperature', temp)
rate = rospy.Rate(10)

# SET UP MESSAGE
temps = temp()

# SET UP HDDTEMP SOCKET
PORT = 7634
HOST = '127.0.0.1'

# SET UP SENSORS
sensors.init()

# THRESHOLDS
CPU_THRESHOLD = 90
SSD_THRESHOLD = 65

# UPDATE TEMPERATURES
def update():
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

    # GET SSD TEMPERATURE
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    data = sock.recv(4096)
    sock.close()
    try:
        temps.ssd = int(data.split('|')[3])
    except IndexError:
        pass

# CHECK THRESHOLDS
def checkup():
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

    if not ok:
       blinky()

# PUBLISH
def publish():
    rospy.loginfo(temps)
    temperature.publish(temps)
    rate.sleep()

# BLINKYTAPE
def blinky():
    colors = []

    for i in range(30):
        colors.append(RGB(255, 255, 255))

    try:
        rospy.wait_for_service('Blinky')
        Blinky = rospy.ServiceProxy('Blinky', BlinkyService)
        res = Blinky(colors, 1)

        if res.success != 0:
            print "Blinky request unsuccessful: %s"%res

    except Exception as e:
        print "Exception: %s"%e

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
