from math import sqrt, pow

SPEED = 340

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

def triangulate2d(DISTANCE, dtx, sdty):
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

DISTANCE = 100
dtx = 232
dty = 232
dtz = 232

print triangulate3d(DISTANCE, dtx, dty, dtz)
print triangulate2d(DISTANCE, dtx, dty)
