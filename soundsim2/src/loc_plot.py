#!/usr/bin/env python
import rospy
#from __future__ import division
from geometry_msgs.msg import Pose2D
from soundsim2.msg import AcousticStatus
import numpy as np
import math
import room_sim
import intersections
import matplotlib.pyplot as plt

def pub_data():
	pub = rospy.Publisher('location', AcousticStatus, queue_size=1)
	#pub = rospy.Publisher('location', Pose2D, queue_size=1)
	rospy.init_node('location_finder', anonymous=False)
	r = rospy.Rate(1)
	


	def locate(room_dim, mic_locs, src_loc):
		fs, i1, i2, i3, i4 = room_sim.sim(room_dim, mic_locs, src_loc, noise=False)#True)
		fs = float(fs)

		t1, t2, t3, t4 = i1/fs, i2/fs, i3/fs, i4/fs
		print("t's: ", t1, t2, t3, t4)
		p1, p2, p3, p4 = mic1_loc[:2], mic2_loc[:2], mic3_loc[:2], mic4_loc[:2]
		s = 343 # speed of sound in m/s
		r1 = 0.05#0.1
		# 14.2 is the largest distance the sound could have possibly traveled
		# ~ sqrt(10^2 + 10^2)
		w = room_dim[0]
		h = room_dim[1]
		fig, ax = plt.subplots()
		ax.set_xlim((0, w))
		ax.set_ylim((0, h))
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
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			    
			ints = intersections.get_intersections(p1, r1, p3, r3)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			ints = intersections.get_intersections(p1, r1, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			"""
			ints = intersections.get_intersections(p2, r2, p3, r3)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			ints = intersections.get_intersections(p2, r2, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

			ints = intersections.get_intersections(p3, r3, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

			if len(valid_ints) >= 3:
				print(str(len(valid_ints)) + " intersections!")
				sorted_ints = PolygonSort(valid_ints)
				area = PolygonArea(sorted_ints)
				print(area)
				if area < 0.3: # 0.4:
					found = True

			plt.gca().set_aspect('equal', adjustable='box')
			#fig.canvas.draw()
			#fig.canvas.flush_events()
			plt.show(block=False)
			plt.pause(0.01)
			#plt.pause(0.5)
			#plt.close()
			#intersections.plot_intersections(p1, r1, p2, r2, p3, r3, p4, r4)
			r1 += 0.03
			c = abs(c - 1)
		found = False
		r1 -= 0.03
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
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			    
			ints = intersections.get_intersections(p1, r1, p3, r3)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			ints = intersections.get_intersections(p1, r1, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			"""
			ints = intersections.get_intersections(p2, r2, p3, r3)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)
			"""
			ints = intersections.get_intersections(p2, r2, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

			ints = intersections.get_intersections(p3, r3, p4, r4)
			if ints is not None:
				for i in ints:
					if (0 <= i[0] <= w) and (0 <= i[1] <= h):
						valid_ints.append(i)
				i1, i2 = ints
				plt.plot([i1[0], i2[0]], [i1[1], i2[1]], '.', color=color)

			if len(valid_ints) >= 3:
				v = np.array(valid_ints)
				cond1 = ((v[:, 0] <= 2).sum() == v.size).astype(np.int)
				cond2 = ((v[:, 0] >= 15.5-2).sum() == v.size).astype(np.int)
				cond3 = ((v[:, 1] <= 2).sum() == v.size).astype(np.int)
				cond4 = ((v[:, 1] <= 14.92-2).sum() == v.size).astype(np.int)
				i = 0
				sum_dist = 0
				while i < len(valid_ints):
					sum_dist += math.sqrt((valid_ints[i][0]-valid_ints[i-1][0])**2 + (valid_ints[i][1]-valid_ints[i-1][1])**2)
					i += 1
				print('hi', cond1, sum_dist)
				"""if (sum_dist <= 0.2) and ((cond1 and cond3) or (cond1 and cond4) or (cond2 and cond3) or (cond2 and cond4)):
					if area < 0.1:
						found = True"""
				if cond1 and (sum_dist >= 0.2):
					valid_ints.append(0, np.mean(v[:, 1]))
				elif cond2 and (sum_dist >= 0.2):
					valid_ints.append(15.5, np.mean(v[:, 1]))
				elif cond3 and (sum_dist >= 0.2):
					valid_ints.append(np.mean(v[:, 0]), 0)
				elif cond4 and (sum_dist >= 0.2):
					valid_ints.append(np.mean(v[:, 0]), 14.92)
				print(str(len(valid_ints)) + " intersections!")
				sorted_ints = PolygonSort(valid_ints)
				area = PolygonArea(sorted_ints)
				print(area)
				print(np.array(valid_ints)[:, 0])

				if area < 0.005: # 0.4:
						found = True

			plt.gca().set_aspect('equal', adjustable='box')
			#fig.canvas.draw()
			#fig.canvas.flush_events()
			plt.show(block=False)
			plt.pause(0.01)
			#plt.pause(0.5)
			#plt.close()
			#intersections.plot_intersections(p1, r1, p2, r2, p3, r3, p4, r4)
			r1 += 0.01
			c = abs(c - 1)

		r1 -= 0.01
		r2 = r1 - (t1 - t2) * s
		r3 = r1 + (t3 - t1) * s
		r4 = r1 + (t4 - t1) * s
		valid_ints = []
		ints = intersections.get_intersections(p1, r1, p2, r2)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)
		    
		ints = intersections.get_intersections(p1, r1, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)
		"""
		ints = intersections.get_intersections(p1, r1, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)
		"""
		"""
		ints = intersections.get_intersections(p2, r2, p3, r3)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)
		"""
		ints = intersections.get_intersections(p2, r2, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)

		ints = intersections.get_intersections(p3, r3, p4, r4)
		if ints is not None:
			for i in ints:
				if (0 <= i[0] <= w) and (0 <= i[1] <= h):
					valid_ints.append(i)
		print("Found points: " + str(valid_ints))
		print("Mean: " + str(np.mean(valid_ints, axis=0)))
		print("Area: " + str(area))
		return(np.mean(valid_ints, axis=0), area)


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


	#room_dim = [10, 10, 3.5]  # meters
	res = 0.02
	w_pix = 775
	h_pix = 746
	width = res*w_pix
	height = res*h_pix
	room_dim = [width, height, 3.5]

	#mics positioned higher give more accurate values for srcs in the middle of the room, less accurate values for srcs closer to corners
	#mics positioned lower give more accurate values for srcs in the corners, less accurate values for srcs near the middle (sources slightly offset from the middle)
	mic1_loc = [0, 0, 1] #[0, 0, 2.5]
	mic2_loc = [width, 0, 1] #[10, 0, 2.5]
	mic3_loc = [0, height, 1] #[0, 10, 2.5]
	mic4_loc = [width, height, 1] #[10, 10, 2.5]

	mic_locs = np.c_[
	    mic1_loc, mic2_loc, mic3_loc, mic4_loc # mic 1  # mic 2 # mic 3
	]
	src_loc = [5, 5, 0]
	source_locations = [[0.1, 0.1, 0],
						[0.5, 6, 0],
						[1, 13, 0],
						[13, 1, 0],
						[7, 1, 0],
						[1, 7, 0],
						[7, 13, 0],
						[13, 7, 0], 
						[6, 8, 0], 
						[4, 4, 0], 
						[6, 4, 0], 
						[1, 0, 0],
						[0, 1, 0],
						[13, 13, 0],
						[5, 5, 0],  
						[6, 6, 0], 
						[4, 6, 0], 
						[2, 9, 0],
						[9, 9, 0]]
	counter = len(source_locations)

	#
	#msg.area = area
	#msg.valid = True
	print("done")
	while not rospy.is_shutdown():
		#src_loc = source_locations[counter % len(source_locations)]
		src_loc = source_locations[0]
		location, area = locate(room_dim, mic_locs, src_loc)
		msg = AcousticStatus()
		pose = Pose2D()
		pose.x = location[0]
		pose.y = location[1]
		#pose.theta = area
		msg.goal = pose
		msg.area = area
		msg.valid = True
		#rospy.loginfo(pose)
		#pub.publish(pose)
		rospy.loginfo(msg)
		pub.publish(msg)
		counter += 1
		print('1')
		rospy.sleep(1.)

if __name__ == '__main__':
	try:
		pub_data()
	except rospy.ROSInterruptException: pass