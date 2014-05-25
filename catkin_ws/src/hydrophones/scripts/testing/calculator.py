from math import sqrt

SOURCE = (x, y, z) = (15,-6.2, 0)
SPEED = 1500

HEIGHT = 1.83
WIDTH = 0.91

MICS = [(0, 0, 10), (WIDTH, 0, 10), (WIDTH, HEIGHT, 10), (0, HEIGHT, 10)]

def magnitude(x, y, z):
    return sqrt(x*x + y*y + z*z)

times = []
for i in range(len(MICS)):
    times.append(magnitude(SOURCE[0] - MICS[i][0], SOURCE[1] - MICS[i][1], SOURCE[2] - MICS[i][2])/SPEED)

for i in range(1, len(MICS)):
    print times[0] - times[i]