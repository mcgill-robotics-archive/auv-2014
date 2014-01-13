#!/usr/bin/env python

from math import pow, sqrt

def triangulate(d, sx, sy, sz):
	try:
		# FIRST SOLUTION
		x0 = (d * d * sy - sx * (d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + pow(sx, 3) * sy - pow(d, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * d * d + pow(sx, 4) * sy * sy * d * d - 2 * pow(sy, 5) * sx * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sx, 3) * pow(sy, 3) * d * d - 4 * sx * sx * sy * sy * pow(d, 4) + 3 * sy * sy * pow(d, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(d, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(d, 4) + sy * sy * pow(sz, 4) * d * d - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + 4 * pow(sy, 4) * sz * sz * d * d + 4 * sx * sx * sy * sy * sz * sz * d * d + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * d * d * sy * sy * sx * pow(sz, 3) + 2 * pow(d, 4) * sz * sy * sy * sx - 2 * d * d * sz * sy * sy * pow(sx, 3) + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * d * d * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) - d * d * sx + sx * sy * sy - sx * sx * sy) / d / sy / 2
		y0 = -(d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + pow(sx, 3) * sy - pow(d, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * d * d + pow(sx, 4) * sy * sy * d * d - 2 * pow(sy, 5) * sx * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sx, 3) * pow(sy, 3) * d * d - 4 * sx * sx * sy * sy * pow(d, 4) + 3 * sy * sy * pow(d, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(d, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(d, 4) + sy * sy * pow(sz, 4) * d * d - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + 4 * pow(sy, 4) * sz * sz * d * d + 4 * sx * sx * sy * sy * sz * sz * d * d + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * d * d * sy * sy * sx * pow(sz, 3) + 2 * pow(d, 4) * sz * sy * sy * sx - 2 * d * d * sz * sy * sy * pow(sx, 3) + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * d * d * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) / d / 2
		z0 = (d * d * sy - sz * (d * d * sz * sz + d * d * sy * sy - d * d * sz * sy - d * d * sy * sx + pow(sx, 3) * sy - pow(d, 4) + sy * pow(sz, 3) - sy * sy * sz * sz + d * d * sx * sx - sx * sx * sy * sy + sqrt(4 * pow(sy, 4) * sx * sx * d * d + pow(sx, 4) * sy * sy * d * d - 2 * pow(sy, 5) * sx * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sx, 3) * pow(sy, 3) * d * d - 4 * sx * sx * sy * sy * pow(d, 4) + 3 * sy * sy * pow(d, 6) - pow(sx, 4) * pow(sy, 4) + 2 * pow(sx, 3) * pow(sy, 5) - pow(sy, 6) * sx * sx - 4 * pow(d, 4) * pow(sy, 4) - pow(sy, 4) * pow(sz, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx - 4 * sy * sy * sz * sz * pow(d, 4) + sy * sy * pow(sz, 4) * d * d - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + 4 * pow(sy, 4) * sz * sz * d * d + 4 * sx * sx * sy * sy * sz * sz * d * d + 2 * pow(sx, 3) * sy * sy * pow(sz, 3) - 2 * d * d * sy * sy * sx * pow(sz, 3) + 2 * pow(d, 4) * sz * sy * sy * sx - 2 * d * d * sz * sy * sy * pow(sx, 3) + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sz * sz - 2 * d * d * pow(sy, 5) * sz)) / (-sy * sy - sx * sx - sz * sz + d * d) - d * d * sz + sy * sy * sz - sy * sz * sz) / d / sy / 2

		# SECOND SOLUTION
		x1 = (d * d * sy + sx * (pow(d, 4) - d * d * sx * sx - d * d * sz * sz + d * d * sx * sy + d * d * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - d * d * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * d * d + pow(sx, 4) * sy * sy * d * d - 4 * sy * sy * sx * sx * pow(d, 4) - 4 * sy * sy * sz * sz * pow(d, 4) - 2 * pow(sy, 3) * pow(sx, 3) * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + pow(sz, 4) * sy * sy * d * d + 4 * pow(sy, 4) * sx * sx * d * d + 3 * sy * sy * pow(d, 6) - 4 * pow(sy, 4) * pow(d, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * d * d - 2 * d * d * pow(sy, 5) * sx - 2 * d * d * pow(sy, 5) * sz + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * d * d * sz * sy * sy * pow(sx, 3) + 2 * pow(d, 4) * sx * sy * sy * sz - 2 * d * d * sx * sy * sy * pow(sz, 3))) / (d * d - sy * sy - sx * sx - sz * sz) - d * d * sx + sy * sy * sx - sy * sx * sx) / d / sy / 2;
		y1 = (pow(d, 4) - d * d * sx * sx - d * d * sz * sz + d * d * sx * sy + d * d * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - d * d * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * d * d + pow(sx, 4) * sy * sy * d * d - 4 * sy * sy * sx * sx * pow(d, 4) - 4 * sy * sy * sz * sz * pow(d, 4) - 2 * pow(sy, 3) * pow(sx, 3) * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + pow(sz, 4) * sy * sy * d * d + 4 * pow(sy, 4) * sx * sx * d * d + 3 * sy * sy * pow(d, 6) - 4 * pow(sy, 4) * pow(d, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * d * d - 2 * d * d * pow(sy, 5) * sx - 2 * d * d * pow(sy, 5) * sz + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * d * d * sz * sy * sy * pow(sx, 3) + 2 * pow(d, 4) * sx * sy * sy * sz - 2 * d * d * sx * sy * sy * pow(sz, 3))) / (d * d - sy * sy - sx * sx - sz * sz) / d / 2;
		z1 = (d * d * sy + sz * (pow(d, 4) - d * d * sx * sx - d * d * sz * sz + d * d * sx * sy + d * d * sz * sy - sy * pow(sz, 3) + sy * sy * sx * sx - sy * pow(sx, 3) + sy * sy * sz * sz - d * d * sy * sy + sqrt(-pow(sy, 4) * pow(sx, 4) - pow(sy, 4) * pow(sz, 4) + 4 * pow(sy, 4) * sz * sz * d * d + pow(sx, 4) * sy * sy * d * d - 4 * sy * sy * sx * sx * pow(d, 4) - 4 * sy * sy * sz * sz * pow(d, 4) - 2 * pow(sy, 3) * pow(sx, 3) * d * d + 2 * pow(sy, 3) * sx * pow(d, 4) - 2 * pow(sy, 3) * pow(sz, 3) * d * d + 2 * pow(sy, 3) * sz * pow(d, 4) + pow(sz, 4) * sy * sy * d * d + 4 * pow(sy, 4) * sx * sx * d * d + 3 * sy * sy * pow(d, 6) - 4 * pow(sy, 4) * pow(d, 4) - pow(sx, 4) * sy * sy * sz * sz - pow(sz, 4) * sy * sy * sx * sx + 4 * sz * sz * sy * sy * sx * sx * d * d - 2 * d * d * pow(sy, 5) * sx - 2 * d * d * pow(sy, 5) * sz + d * d * pow(sy, 6) + 2 * pow(sy, 5) * pow(sx, 3) + 2 * pow(sy, 5) * pow(sz, 3) - pow(sy, 6) * sx * sx - pow(sy, 6) * sz * sz + 2 * sy * sy * pow(sz, 3) * pow(sx, 3) - 2 * d * d * sz * sy * sy * pow(sx, 3) + 2 * pow(d, 4) * sx * sy * sy * sz - 2 * d * d * sx * sy * sy * pow(sz, 3))) / (d * d - sy * sy - sx * sx - sz * sz) - d * d * sz + sy * sy * sz - sy * sz * sz) / d / sy / 2;

		coordinates = [[x0, y0, z0], [x1, y1, z1]]

		return coordinates

	except:
		return 'degenerate triangle'

d = 100
sx = -312
sy = -323
sz = -321

print triangulate(d, sx, sy, sz)