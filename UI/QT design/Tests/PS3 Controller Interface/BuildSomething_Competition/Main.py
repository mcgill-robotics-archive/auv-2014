#!usr/bin/python
'''
Created on Aug 10th 2013

@author: Jean-Sebastien Dery and Alan Yang
'''

import PS3ControllerOriginal
import time
import pygame


updateFrequency = 5


class Main(object):
    def __init__(self):
        self.ps3 = PS3ControllerOriginal.PS3Controller()

    def run(self):
        while True:
            time.sleep(1 / updateFrequency)
            self.ps3.updateController()
            print self.ps3.getRightStickData()


main = Main()
main.run()
