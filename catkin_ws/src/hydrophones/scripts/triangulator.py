import math

def triangulate(d, sx, sy, sz):
	x = (d * d * sy - sx * (d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + math.pow(sx, 0.3e1) * sy - math.pow(d, 0.4e1) + sy * math.pow(sz, 0.3e1) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + math.sqrt(0.4e1 * math.pow(sy, 0.4e1) * sx * sx * d * d + math.pow(sx, 0.4e1) * sy * sy * d * d - 0.2e1 * math.pow(sy, 0.5e1) * sx * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sx * math.pow(d, 0.4e1) - 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.3e1) * d * d - 0.4e1 * sx * sx * sy * sy * math.pow(d, 0.4e1) + 0.3e1 * sy * sy * math.pow(d, 0.6e1) - math.pow(sx, 0.4e1) * math.pow(sy, 0.4e1) + 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.5e1) - math.pow(sy, 0.6e1) * sx * sx - 0.4e1 * math.pow(d, 0.4e1) * math.pow(sy, 0.4e1) - math.pow(sy, 0.4e1) * math.pow(sz, 0.4e1) - math.pow(sx, 0.4e1) * sy * sy * sz * sz - math.pow(sz, 0.4e1) * sy * sy * sx * sx - 0.4e1 * sy * sy * sz * sz * math.pow(d, 0.4e1) + sy * sy * math.pow(sz, 0.4e1) * d * d - 0.2e1 * math.pow(sy, 0.3e1) * math.pow(sz, 0.3e1) * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sz * math.pow(d, 0.4e1) + 0.4e1 * math.pow(sy, 0.4e1) * sz * sz * d * d + 0.4e1 * sx * sx * sy * sy * sz * sz * d * d + 0.2e1 * math.pow(sx, 0.3e1) * sy * sy * math.pow(sz, 0.3e1) - 0.2e1 * d * d * sy * sy * sx * math.pow(sz, 0.3e1) + 0.2e1 * math.pow(d, 0.4e1) * sz * sy * sy * sx - 0.2e1 * d * d * sz * sy * sy * math.pow(sx, 0.3e1) + d * d * math.pow(sy, 0.6e1) + 0.2e1 * math.pow(sy, 0.5e1) * math.pow(sz, 0.3e1) - math.pow(sy, 0.6e1) * sz * sz - 0.2e1 * d * d * math.pow(sy, 0.5e1) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) - d * d * sx + sx * sy * sy - sx * sx * sy) / d / sy / 0.2e1
	y = -(d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + math.pow(sx, 0.3e1) * sy - math.pow(d, 0.4e1) + sy * math.pow(sz, 0.3e1) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + math.sqrt(0.4e1 * math.pow(sy, 0.4e1) * sx * sx * d * d + math.pow(sx, 0.4e1) * sy * sy * d * d - 0.2e1 * math.pow(sy, 0.5e1) * sx * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sx * math.pow(d, 0.4e1) - 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.3e1) * d * d - 0.4e1 * sx * sx * sy * sy * math.pow(d, 0.4e1) + 0.3e1 * sy * sy * math.pow(d, 0.6e1) - math.pow(sx, 0.4e1) * math.pow(sy, 0.4e1) + 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.5e1) - math.pow(sy, 0.6e1) * sx * sx - 0.4e1 * math.pow(d, 0.4e1) * math.pow(sy, 0.4e1) - math.pow(sy, 0.4e1) * math.pow(sz, 0.4e1) - math.pow(sx, 0.4e1) * sy * sy * sz * sz - math.pow(sz, 0.4e1) * sy * sy * sx * sx - 0.4e1 * sy * sy * sz * sz * math.pow(d, 0.4e1) + sy * sy * math.pow(sz, 0.4e1) * d * d - 0.2e1 * math.pow(sy, 0.3e1) * math.pow(sz, 0.3e1) * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sz * math.pow(d, 0.4e1) + 0.4e1 * math.pow(sy, 0.4e1) * sz * sz * d * d + 0.4e1 * sx * sx * sy * sy * sz * sz * d * d + 0.2e1 * math.pow(sx, 0.3e1) * sy * sy * math.pow(sz, 0.3e1) - 0.2e1 * d * d * sy * sy * sx * math.pow(sz, 0.3e1) + 0.2e1 * math.pow(d, 0.4e1) * sz * sy * sy * sx - 0.2e1 * d * d * sz * sy * sy * math.pow(sx, 0.3e1) + d * d * math.pow(sy, 0.6e1) + 0.2e1 * math.pow(sy, 0.5e1) * math.pow(sz, 0.3e1) - math.pow(sy, 0.6e1) * sz * sz - 0.2e1 * d * d * math.pow(sy, 0.5e1) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) / d / 0.2e1
	z = (d * d * sy - sz * (d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + math.pow(sx, 0.3e1) * sy - math.pow(d, 0.4e1) + sy * math.pow(sz, 0.3e1) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + math.sqrt(0.4e1 * math.pow(sy, 0.4e1) * sx * sx * d * d + math.pow(sx, 0.4e1) * sy * sy * d * d - 0.2e1 * math.pow(sy, 0.5e1) * sx * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sx * math.pow(d, 0.4e1) - 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.3e1) * d * d - 0.4e1 * sx * sx * sy * sy * math.pow(d, 0.4e1) + 0.3e1 * sy * sy * math.pow(d, 0.6e1) - math.pow(sx, 0.4e1) * math.pow(sy, 0.4e1) + 0.2e1 * math.pow(sx, 0.3e1) * math.pow(sy, 0.5e1) - math.pow(sy, 0.6e1) * sx * sx - 0.4e1 * math.pow(d, 0.4e1) * math.pow(sy, 0.4e1) - math.pow(sy, 0.4e1) * math.pow(sz, 0.4e1) - math.pow(sx, 0.4e1) * sy * sy * sz * sz - math.pow(sz, 0.4e1) * sy * sy * sx * sx - 0.4e1 * sy * sy * sz * sz * math.pow(d, 0.4e1) + sy * sy * math.pow(sz, 0.4e1) * d * d - 0.2e1 * math.pow(sy, 0.3e1) * math.pow(sz, 0.3e1) * d * d + 0.2e1 * math.pow(sy, 0.3e1) * sz * math.pow(d, 0.4e1) + 0.4e1 * math.pow(sy, 0.4e1) * sz * sz * d * d + 0.4e1 * sx * sx * sy * sy * sz * sz * d * d + 0.2e1 * math.pow(sx, 0.3e1) * sy * sy * math.pow(sz, 0.3e1) - 0.2e1 * d * d * sy * sy * sx * math.pow(sz, 0.3e1) + 0.2e1 * math.pow(d, 0.4e1) * sz * sy * sy * sx - 0.2e1 * d * d * sz * sy * sy * math.pow(sx, 0.3e1) + d * d * math.pow(sy, 0.6e1) + 0.2e1 * math.pow(sy, 0.5e1) * math.pow(sz, 0.3e1) - math.pow(sy, 0.6e1) * sz * sz - 0.2e1 * d * d * math.pow(sy, 0.5e1) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) - d * d * sz + sy * sy * sz - sy * sz * sz) / d / sy / 0.2e1

	coordinates = [x, y, z]

	return coordinates

d = 200
sx = 1
sy = 2
sz = 3

print triangulate(d, sx, sy, sz)
