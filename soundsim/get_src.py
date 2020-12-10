from __future__ import division
import numpy as np
import room_sim
import intersections
import matplotlib.pyplot as plt

src_loc = [5, 5, 0]

mic1_loc = [0, 0, 0] #[0, 0, 2.5]
mic2_loc = [10, 0, 0] #[10, 0, 2.5]
mic3_loc = [0, 10, 0] #[0, 10, 2.5]
mic4_loc = [10, 10, 0] #[10, 10, 2.5]

mic_locs = np.c_[
    mic1_loc, mic2_loc, mic3_loc, mic4_loc # mic 1  # mic 2 # mic 3
]

fs, i1, i2, i3, i4 = room_sim.sim(src_loc, mic_locs, noise=True)

t1, t2, t3, t4 = i1/fs, i2/fs, i3/fs, i4/fs
print("t's: ", t1, t2, t3, t4)
p1, p2, p3, p4 = mic1_loc[:2], mic2_loc[:2], mic3_loc[:2], mic4_loc[:2]
s = 343 # speed of sound in m/s

def locate(p1, p2, p3, p4):
	r1 = 0.1#0.1
	# 14.2 is the largest distance the sound could have possibly traveled
	# ~ sqrt(10^2 + 10^2)
	fig, ax = plt.subplots()
	ax.set_xlim((0, 10))
	ax.set_ylim((0, 10))
	plt.plot(src_loc[0], src_loc[1], '.', color='g')
	c = 1
	found = False
	x1, y1 = p1[0], p1[1]
	x2, y2 = p2[0], p2[1]
	x3, y3 = p3[0], p3[1]
	x4, y4 = p4[0], p4[1]
	#area = 10000000
	while not found:
	#while r1 <= 14.2:
		if c == 1:
			color = 'r'
		else:
			color = 'black'
		r2 = r1 - (t1 - t2) * s
		r3 = r1 + (t3 - t1) * s
		r4 = r1 + (t4 - t1) * s
		

		circle1 = plt.Circle((x1, y1), r1, color='b', fill=False)
		circle2 = plt.Circle((x2, y2), r2, color='b', fill=False)
		circle3 = plt.Circle((x3, y3), r3, color='b', fill=False)
		circle4 = plt.Circle((x4, y4), r4, color='b', fill=False)
		
		ax.add_artist(circle1)
		ax.add_artist(circle2)
		ax.add_artist(circle3)
		ax.add_artist(circle4)

		valid_ints = []

		ints = intersections.get_intersections(p1, r1, p2, r2)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
		    
		ints = intersections.get_intersections(p1, r1, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

		ints = intersections.get_intersections(p1, r1, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

		ints = intersections.get_intersections(p2, r2, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

		ints = intersections.get_intersections(p2, r2, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

		ints = intersections.get_intersections(p3, r3, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
			i1, i2 = ints
			plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

		if len(valid_ints) > 3:
			print(str(len(valid_ints)) + " intersections!")
			sorted_ints = PolygonSort(valid_ints)
			area = PolygonArea(sorted_ints)
			print(area)
			if area < 0.1: # 0.4:
				found = True

		plt.gca().set_aspect('equal', adjustable='box')
		#fig.canvas.draw()
		#fig.canvas.flush_events()
		plt.show(block=False)
		plt.pause(0.01)
		#plt.pause(0.001)
		#plt.close()
		#intersections.plot_intersections(p1, r1, p2, r2, p3, r3, p4, r4)
		r1 += 0.1
		c = abs(c - 1)
	currarea = area
	while currarea <= area:
		print(currarea, area)
		area = currarea
		r2 = r1 - (t1 - t2) * s
		r3 = r1 + (t3 - t1) * s
		r4 = r1 + (t4 - t1) * s
		valid_ints = []
		ints = intersections.get_intersections(p1, r1, p2, r2)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)
		    
		ints = intersections.get_intersections(p1, r1, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)

		ints = intersections.get_intersections(p1, r1, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)

		ints = intersections.get_intersections(p2, r2, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)

		ints = intersections.get_intersections(p2, r2, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)

		ints = intersections.get_intersections(p3, r3, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
					valid_ints.append(i)

		print(str(len(valid_ints)) + " intersections!")
		sorted_ints = PolygonSort(valid_ints)
		currarea = PolygonArea(sorted_ints)
		print(area)
		r1 += 0.001
		print('one iter')
	r1 -= 0.01
	r2 = r1 - (t1 - t2) * s
	r3 = r1 + (t3 - t1) * s
	r4 = r1 + (t4 - t1) * s
	valid_ints = []
	ints = intersections.get_intersections(p1, r1, p2, r2)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)
	    
	ints = intersections.get_intersections(p1, r1, p3, r3)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)

	ints = intersections.get_intersections(p1, r1, p4, r4)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)

	ints = intersections.get_intersections(p2, r2, p3, r3)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)

	ints = intersections.get_intersections(p2, r2, p4, r4)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)

	ints = intersections.get_intersections(p3, r3, p4, r4)
	if ints is not None:
		for i in ints:
			if (0 <= i[0] <= 10) and (0 <= i[1] <= 10):
				valid_ints.append(i)
	print("Found points: " + str(valid_ints))
	print("Mean: " + str(np.mean(valid_ints, axis=0)))
	print("Area: " + str(area))


#https://plotly.com/python/v3/polygon-area/
def PolygonSort(corners):
    n = len(corners)
    cx = float(sum(x for x, y in corners)) / n
    cy = float(sum(y for x, y in corners)) / n
    cornersWithAngles = []
    for x, y in corners:
        an = (np.arctan2(y - cy, x - cx) + 2.0 * np.pi) % (2.0 * np.pi)
        cornersWithAngles.append((x, y, an))
    cornersWithAngles.sort(key = lambda tup: tup[2])
    return map(lambda (x, y, an): (x, y), cornersWithAngles)

def PolygonArea(corners):
    n = len(corners)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += corners[i][0] * corners[j][1]
        area -= corners[j][0] * corners[i][1]
    area = abs(area) / 2.0
    return area


locate(p1, p2, p3, p4)
print("done")