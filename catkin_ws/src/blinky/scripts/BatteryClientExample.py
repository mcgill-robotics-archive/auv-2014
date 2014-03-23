#!/usr/bin/env python

from random import randint
import time

from BlinkyAPI import *

def generate_colors(n):
	colors = []
	t = n

	for i in range(30):
		if n % 2 == 0:
			colors.append(RGB(0,0,0))
		else:
			colors.append(RGB(randint(0,255),randint(0,255),randint(0,255)))

		n = n / 2

	return colors

a = 0
while True:	
	Battery_sendColors(generate_colors(a))
	time.sleep(0.1)
	a = a + 1
