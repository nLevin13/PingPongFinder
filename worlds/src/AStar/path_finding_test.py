#!/usr/bin/env python
from sys import argv
import rospy
from worlds.srv import MapAndEndpts, MapAndEndptsResponse
from geometry_msgs.msg import Pose2D

def path_finding_client(s, g, img_file):
	print('hi1')
	rospy.wait_for_service('path_find')
	print('hi2')
	try:
		find_path = rospy.ServiceProxy('path_find', MapAndEndpts)
		response = find_path(Pose2D(s[0], s[1], 0), Pose2D(g[0], g[1], 0), img_file)
		# return list(map(lambda pose: [pose.x, pose.y], response.cornerpts))
		return response
	except rospy.ServiceException as e:
	    print("Service call failed: %s"%e)

if __name__ == "__main__":
	cornerpts = path_finding_client([float(argv[2]), float(argv[3])], [float(argv[4]), float(argv[5])], argv[1])
	print('response', cornerpts)
