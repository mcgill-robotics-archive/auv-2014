#!/usr/bin/env python
import rospy
from status.msg import usb

import re
import subprocess


def get_ports():
    device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
    df = subprocess.check_output("lsusb", shell=True)
    num_device = 0
    ports = []
    names = []
    for i in df.split('\n'):
        if i:
            info = device_re.match(i)
            if info:
                dinfo = info.groupdict()
                dinfo['device']='/dev/bus/usb/%s/%s'%(dinfo.pop('bus'), dinfo.pop('device'))
                ports.append(dinfo['device'])
                names.append(dinfo['tag'])
                num_device+=1
    #print num_device
    #print ports
    #print names
    msg = usb(num_device, ports, names)
    return msg

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            rospy.init_node('USB_monitor')
            usb_topic = rospy.Publisher('status/usb', usb)
            rate = rospy.Rate(1)

            while True:
                try:
                    msg = get_ports()
                    #print msg
                    usb_topic.publish(msg)
                    rate.sleep()

                except KeyboardInterrupt:
                    break
    except Exception as e:
        print e