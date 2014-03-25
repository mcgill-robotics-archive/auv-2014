from BlinkyTape import BlinkyTape
import optparse
import math
from time import sleep

port="/dev/blinkyTape"
blinky=BlinkyTape(port)
delayTime= 0.01

x=0
while True:
    for y in range(0,60):
        blinky.sendPixel(8*((x+y)%30),4*((x+y)%60),2*((x+y)%120))  
    blinky.show()
    x+=1
    sleep(delayTime)


