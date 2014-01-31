#!/usr/bin/env python

from math import sqrt, pow, pi, sin, cos
from numpy import linalg

SPEED = 330

def magnitude(x, y, z):
        return sqrt(x * x + y * y + z * z)

def magnitude(x, y):
        return sqrt(x * x + y * y)

def triangulate3d(DISTANCE, dtx, dty, dtz):
    try:
        # DISTANCE DIFFERENCES (ds = v * dt)
        sx = dtx * SPEED
        sy = dty * SPEED
        sz = dtz * SPEED

        # FIRST SOLUTION
        x0 = (DISTANCE * DISTANCE * sy - sx * (DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) - DISTANCE * DISTANCE * sx + sx * sy * sy - sx * sx * sy) / DISTANCE / sy / 2
        y0 = -(DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) / DISTANCE / 2
        z0 = (DISTANCE * DISTANCE * sy - sz * (DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sy * sy - DISTANCE * DISTANCE * sz * sy - DISTANCE * DISTANCE * sy * sx + pow(sx, 3) * sy - pow(DISTANCE, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + DISTANCE * DISTANCE * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sy, 5) * sx * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE - 4 * sx * sx * sy * sy * pow(DISTANCE, 4) + 3 * sy * sy * pow(DISTANCE, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(DISTANCE, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) + sy * sy * pow(sz, 4) * DISTANCE * DISTANCE - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + 4 * sx * sx * sy * sy * sz * sz * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * DISTANCE * DISTANCE * sy * sy * sx * pow(sz, 3) + 2 * pow(DISTANCE, 4) * sz * sy * sy * sx - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + DISTANCE * DISTANCE) - DISTANCE * DISTANCE * sz + sy * sy * sz - sy * sz * sz) / DISTANCE / sy / 2

        # SECOND SOLUTION
        x1 = (DISTANCE * DISTANCE * sy + sx * (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) - DISTANCE * DISTANCE * sx + sy * sy * sx - sy * sx * sx) / DISTANCE / sy / 2;
        y1 = (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) / DISTANCE / 2;
        z1 = (DISTANCE * DISTANCE * sy + sz * (pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx - DISTANCE * DISTANCE * sz * sz + DISTANCE * DISTANCE * sx * sy + DISTANCE * DISTANCE * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - DISTANCE * DISTANCE * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * DISTANCE * DISTANCE + pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 4 * sy * sy * sx * sx * pow(DISTANCE, 4) - 4 * sy * sy * sz * sz * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sx, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sx * pow(DISTANCE, 4) - 2 * pow(sy, 3) * pow(sz, 3) * DISTANCE * DISTANCE + 2 * pow(sy, 3) * sz * pow(DISTANCE, 4) + pow(sz, 4) * sy * sy * DISTANCE * DISTANCE + 4 * pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 3 * sy * sy * pow(DISTANCE, 6) - 4 * pow(sy, 4) * pow(DISTANCE, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * DISTANCE * DISTANCE - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sx - 2 * DISTANCE * DISTANCE * pow(sy, 5) * sz + DISTANCE * DISTANCE * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * DISTANCE * DISTANCE * sz * sy * sy * pow(sx, 3) + 2 * pow(DISTANCE, 4) * sx * sy * sy * sz - 2 * DISTANCE * DISTANCE * sx * sy * sy * pow(sz, 3))) / (DISTANCE * DISTANCE - sy * sy - sx * sx - sz * sz) - DISTANCE * DISTANCE * sz + sy * sy * sz - sy * sz * sz) / DISTANCE / sy / 2;

        # SELECT CORRECT SOLUTION
        minTime = min(dtx, min(dty, dtz))
        if dtx == minTime:
            if magnitude(x0 - DISTANCE, y0, z0) < magnitude(x1 - DISTANCE, y1, z1):
                x = x0
                y = y0
                z = z0
            else:
                x = x1
                y = y1
                z = z1
        elif dty == minTime:
            if magnitude(x0, y0 - DISTANCE, z0) < magnitude(x1, y1 - DISTANCE, z1):
                x = x0
                y = y0
                z = z0
            else:
                x = x1
                y = y1
                z = z1
        else:
            if magnitude(x0, y0, z0 - DISTANCE) < magnitude(x1, y1, z1 - DISTANCE):
                x = x0
                y = y0
                z = z0
            else:
                x = x1
                y = y1
                z = z1

        coordinates = [x, y, z]

        return coordinates

    except:
        return 'degenerate triangle'

def triangulate2d(DISTANCE, dtx, dty):
    try:
        # DISTANCE DIFFERENCES (ds = v * dt)
        sx = dtx * SPEED
        sy = dty * SPEED

        # FIRST SOLUTION
        x0 = (DISTANCE * DISTANCE * sy * sx - DISTANCE * DISTANCE * sy * sy + sx * sx * sy * sy - pow(sy, 3) * sx + pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx + sqrt(2 * sx * sx * pow(DISTANCE, 6) - 3 * pow(sx, 4) * pow(DISTANCE, 4) - 3 * sx * sx * sy * sy * pow(DISTANCE, 4) + pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 4 * pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * pow(DISTANCE, 4) - pow(sx, 4) * pow(sy, 4) - 2 * pow(sx, 5) * DISTANCE * DISTANCE * sy + pow(sx, 6) * DISTANCE * DISTANCE + 2 * pow(sx, 5) * pow(sy, 3) - pow(sx, 6) * sy * sy)) / (DISTANCE * DISTANCE - sy * sy - sx * sx) / DISTANCE / 2;
        y0 = (DISTANCE * DISTANCE * sx + sy * (DISTANCE * DISTANCE * sy * sx - DISTANCE * DISTANCE * sy * sy + sx * sx * sy * sy - pow(sy, 3) * sx + pow(DISTANCE, 4) - DISTANCE * DISTANCE * sx * sx + sqrt(2 * sx * sx * pow(DISTANCE, 6) - 3 * pow(sx, 4) * pow(DISTANCE, 4) - 3 * sx * sx * sy * sy * pow(DISTANCE, 4) + pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 4 * pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * pow(DISTANCE, 4) - pow(sx, 4) * pow(sy, 4) - 2 * pow(sx, 5) * DISTANCE * DISTANCE * sy + pow(sx, 6) * DISTANCE * DISTANCE + 2 * pow(sx, 5) * pow(sy, 3) - pow(sx, 6) * sy * sy)) / (DISTANCE * DISTANCE - sy * sy - sx * sx) - DISTANCE * DISTANCE * sy + sx * sx * sy - sx * sy * sy) / DISTANCE / sx / 2;

        # SECOND SOLUTION
        x1 = -(-DISTANCE * DISTANCE * sy * sx + DISTANCE * DISTANCE * sy * sy - sx * sx * sy * sy + pow(sy, 3) * sx - pow(DISTANCE, 4) + DISTANCE * DISTANCE * sx * sx + sqrt(2 * sx * sx * pow(DISTANCE, 6) - 3 * pow(sx, 4) * pow(DISTANCE, 4) - 3 * sx * sx * sy * sy * pow(DISTANCE, 4) + pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 4 * pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * pow(DISTANCE, 4) - pow(sx, 4) * pow(sy, 4) - 2 * pow(sx, 5) * DISTANCE * DISTANCE * sy + pow(sx, 6) * DISTANCE * DISTANCE + 2 * pow(sx, 5) * pow(sy, 3) - pow(sx, 6) * sy * sy)) / (DISTANCE * DISTANCE - sy * sy - sx * sx) / DISTANCE / 2;
        y1 = (DISTANCE * DISTANCE * sx - sy * (-DISTANCE * DISTANCE * sy * sx + DISTANCE * DISTANCE * sy * sy - sx * sx * sy * sy + pow(sy, 3) * sx - pow(DISTANCE, 4) + DISTANCE * DISTANCE * sx * sx + sqrt(2 * sx * sx * pow(DISTANCE, 6) - 3 * pow(sx, 4) * pow(DISTANCE, 4) - 3 * sx * sx * sy * sy * pow(DISTANCE, 4) + pow(sy, 4) * sx * sx * DISTANCE * DISTANCE + 4 * pow(sx, 4) * sy * sy * DISTANCE * DISTANCE - 2 * pow(sx, 3) * pow(sy, 3) * DISTANCE * DISTANCE + 2 * pow(sx, 3) * sy * pow(DISTANCE, 4) - pow(sx, 4) * pow(sy, 4) - 2 * pow(sx, 5) * DISTANCE * DISTANCE * sy + pow(sx, 6) * DISTANCE * DISTANCE + 2 * pow(sx, 5) * pow(sy, 3) - pow(sx, 6) * sy * sy)) / (DISTANCE * DISTANCE - sy * sy - sx * sx) - DISTANCE * DISTANCE * sy + sx * sx * sy - sx * sy * sy) / DISTANCE / sx / 2;

        # SELECT CORRECT SOLUTION
        minTime = min(dtx, dty)
        if dtx == minTime:
            if magnitude(x0 - DISTANCE, y0) < magnitude(x1 - DISTANCE, y1):
                x = x0
                y = y0
            else:
                x = x1
                y = y1
        else:
            if magnitude(x0, y0 - DISTANCE) < magnitude(x1, y1 - DISTANCE):
                x = x0
                y = y0
            else:
                x = x1
                y = y1


        coordinates = [x, y]

        return coordinates

    except:
        return 'degenerate triangle'

def triangulate(dtx, dty):
    DISTANCE_X = 499.1  # Distance M1 <---> M3
    DISTANCE_Y = 1267.9 # Distance M2 <---> M3

    sx = dtx*SPEED # Distance M1 <---> M2
    sy = dty*SPEED # Distance M1 <---> M3

    phi = 16.177*pi/180 # Angle between M1 and M2 in rads

    # Initial Guess
    r1 = 1000
    theta = 0

    # Solution vector
    Y = [r1, theta]

    # Position Vector
    R = [ Y[0], sx, sy, DISTANCE_X, DISTANCE_Y ]
    F = [0,0]
    J = [[0,0],[0,0]]

    # Tolerance
    t = 1e-9
    e = 100

    # Iteration count
    i = 0

    # As long as the error is higher than the tolerance
    while e > t:
        R = [ Y[0], sx, sy, DISTANCE_X, DISTANCE_Y ]
        theta = Y[1]

        # Calculate the function
        F = [(R[0] + R[2])**2 - (R[0] + R[1])**2 + R[3]**2 - 2*(R[0] + R[2]) * R[3] * cos(theta),
            (R[0] + R[2])**2 + R[4]**2 - R[0]**2 - 2 * (R[0]+R[2]) * R[4] * cos(theta-phi)]
        J[0][0] = 2 * R[2] - 2 * R[1] - 2 * R[3] * cos(theta)
        J[0][1] = 2 * (R[0] + R[2]) * R[3] * sin(theta)
        J[1][0] = 2 * R[2] - 2 * R[4] * cos(theta - phi)
        J[1][1] = 2 * (R[0] + R[2]) * R[4] * sin(theta - phi)
        
        delta_x = -linalg.solve(J, F)
        
        Y += delta_x
        
        e = sqrt(delta_x[0]**2 + delta_x[1]**2)
        
        if i > 1000:
            print "****Did not converge within", i, " iterations.**** with e = ", e
            e = 0
        
        i += 1

    return Y



DISTANCE = 1200
dtx = 0.455
dty = 0.606
dtz = 0.606

print '3D Explicit: ', triangulate3d(DISTANCE, dtx, dty, dtz)
print '2D Explicit: ', triangulate2d(DISTANCE, dtx, dty)
print 'Numerical Method: ', triangulate(dtx, dty)
