#!/usr/bin/env python

from math import sqrt

SPEED = 340
DISTANCE = 0.016764


def calculate(x, y):
    return (sqrt((DISTANCE / 2 + x**2)**2 + y**2) - sqrt((x - DISTANCE / 2)**2 + y**2)) / SPEED

print calculate(0.20, 0.30)
