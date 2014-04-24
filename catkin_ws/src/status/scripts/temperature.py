#!/usr/bin/env python

# IMPORTS
import sensors
import roslib; roslib.load_manifest('status')
import rospy
import blinky
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
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

# THRESHOLDS
CPU_THRESHOLD = 90
SSD_THRESHOLD = 65

# UPDATE TEMPERATURES
def update():
    # GET CPU CORE TEMPERATURES
    for chip in sensors.iter_detected_chips():
        for feature in chip:
            if feature.name.startswith('Core'):
                if feature.name.endswith('0'):
                    temps.core_0 = feature.get_value()
                elif feature.name.endswith('1'):
                    temps.core_1 = feature.get_value()
                elif feature.name.endswith('2'):
                    temps.core_2 = feature.get_value()
                elif feature.name.endswith('3'):
                    temps.core_3 = feature.get_value()

    # GET SSD TEMPERATURE
    data = sock.recv(4096)
    temps.ssd = int(data.split('|')[3])

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
        Battery_sendColors(generate_colors(255))

# PUBLISH
def publish():
    rospy.loginfo(temps)
    temp.publish(temps)
    rate.sleep(10)

# BLINKYTAPE
def generate_colors(n):
    colors = []

    for i in range(30):
        colors.append(RGB(n, n, n))

    return colors

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
            sock.close()
            break
