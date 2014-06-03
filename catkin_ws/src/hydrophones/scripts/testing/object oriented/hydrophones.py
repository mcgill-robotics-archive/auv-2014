#!/usr/bin/env python

### IMPORTS
import numpy as np
from objects import *
# from inputs import real
from tdoa import gccphat as tdoa
from solve import multilateration as solver
from outputs import screen as out

### PARAMETERS
SPEED = 1500                # SPEED OF SOUND IN MEDIUM      m/s
SAMPLING_FREQUENCY = 192e3  # SAMPLING FREQUENCY OF SIGNAL  Hz
TARGET_FREQUENCY = 30e3     # FREQUENCY OF PINGER           Hz

### VARIABLES
NUMBER_OF_MICS = 4          # RECEIVERS CONNECTED
HEIGHT = 1.83               # HEIGHT OF RECEIVER ARRAY      m
WIDTH = 0.91                # WIDTH OF RECEIVER ARRAY       m
DEPTH_OF_PINGER = 4.2672    # DEPTH OF PINGER FROM SURFACE  m

### CREATE RECEIVER ARRAY 
ARRAY = [Point(0,0), Point(WIDTH,0), Point(WIDTH,HEIGHT), Point(0,HEIGHT)]
mics = [Mic(ARRAY[i]) for i in range(NUMBER_OF_MICS)]

### GET TDOA
TIMES = [0, 0.000487006198855, -0.000240550990029, -0.000722579368756]
for i in range(NUMBER_OF_MICS):
    mics[i].diff = TIMES[i]

### SOLVE
sol = solver.solve(mics,SPEED)

### OUTPUT
out.send(sol)