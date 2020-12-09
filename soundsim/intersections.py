import math
import numpy as np
import matplotlib.pyplot as plt


# gets intersections of 2 circles if there are any, otherwise returns None
# see here:
# http://paulbourke.net/geometry/circlesphere/
# https://stackoverflow.com/questions/55816902/finding-the-intersection-of-two-circles
def get_intersections(p0, r0, p1, r1):
	# circle 1: p0 = (x0, y0), radius r0
	# circle 2: p1 = (x1, y1), radius r1
	x0, y0 = p0[0], p0[1]
	x1, y1 = p1[0], p1[1]

	#distance between center of 2 circles
	dist = math.sqrt((x1-x0)**2 + (y1-y0)**2)

	# circles are farther apart than the sum of radius's
	if dist > r0 + r1:
		return None
	# circles are within each other
	if dist < abs(r0 - r1):
		return None
	# circles are coincident, doesn't make sense to store a value
	if dist == 0 and r0 == r1:
		return None
	a = (r0**2 - r1**2 + dist**2) / (2 * dist)
	h = math.sqrt(r0**2 - a**2)
	#making a separate point x2, y2 now
	x2 = x0 + (a*(x1-x0)/dist)
	y2 = y0 + (a*(y1-y0)/dist)

	x3 = x2 + (h*(y1-y0)/dist)
	y3 = y2 - (h*(x1-x0)/dist)

	x4 = x2 - (h*(y1-y0)/dist)
	y4 = y2 + (h*(x1-x0)/dist)

	p3 = [x3, y3]
	p4 = [x4, y4]

	return p3, p4

def plot_intersections(p0, r0, p1, r1, p2, r2, p3, r3):
	x0, y0 = p0[0], p0[1]
	x1, y1 = p1[0], p1[1]
	x2, y2 = p2[0], p2[1]
	x3, y3 = p3[0], p3[1]

	circle1 = plt.Circle((x0, y0), r0, color='b', fill=False)
	circle2 = plt.Circle((x1, y1), r1, color='b', fill=False)
	circle3 = plt.Circle((x2, y2), r2, color='b', fill=False)
	circle4 = plt.Circle((x3, y3), r3, color='b', fill=False)

	fig, ax = plt.subplots() 
	ax.set_xlim((0, 10))
	ax.set_ylim((0, 10))
	ax.add_artist(circle1)
	ax.add_artist(circle2)
	ax.add_artist(circle3)
	ax.add_artist(circle4)

	intersections = get_intersections(p0, r0, p1, r1)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')
	    
	intersections = get_intersections(p0, r0, p2, r2)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')

	intersections = get_intersections(p0, r0, p3, r3)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')

	intersections = get_intersections(p1, r1, p2, r2)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')

	intersections = get_intersections(p1, r1, p3, r3)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')

	intersections = get_intersections(p2, r2, p3, r3)
	print(intersections)
	if intersections is not None:
	    i1, i2 = intersections 
	    plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color='r')

	plt.gca().set_aspect('equal', adjustable='box')
	plt.show(block=False)
	plt.pause(0.1)
	plt.close()
"""
p0, r0 = [0, 0], 5
p1, r1 = [2, 2], 5
p2, r2 = [-1, 0], 2.5
p3, r3 = [5, -5], 4

plot_intersections(p0, r0, p1, r1, p2, r2, p3, r3)
"""