### IMPORTS
from math import sqrt, atan2

class Point(object):
    ''' 2D Point class '''
    __slots__ = ['x', 'y', 'r', 'theta']

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.r = sqrt(x**2 + y**2)
        self.theta = atan2(y,x)
