from BlinkyTape import BlinkyTape
import optparse
import math
from time import sleep

port="/dev/blinkyTape"
blinky=BlinkyTape(port)
delayTime= 0.1

x=0
while True:
    for y in range(0,60):
        blinky.sendPixel((x+y)%255,(x+y)%255,(x+y)%255)  
    blinky.show()
    x+=8
    sleep(delayTime)


