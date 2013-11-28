#!usr/bin/python
'''
Created on Aug 10th 2013

@author: Jean-Sebastien Dery and Alan Yang
'''

import PS3Controller
import time


updateFrequency = 5


class Main(object):
    def __init__(self):
        self.ps3 = PS3Controller.PS3Controller()

    def run(self):
        while True:
            time.sleep(1 / updateFrequency)
            self.ps3.updateController()
            buttonState=self.ps3.returnButtons()
            print buttonState
            print self.ps3.lJoyX, self.ps3.lJoyY



main = Main()
main.run()
