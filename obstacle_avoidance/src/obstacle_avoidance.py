#!/usr/bin/env python
import rospy, png, math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np

class RobotObsDetect:
	def __init__(self, robot_name, map_path):

		""" Metadata """
		self.robot_name = robot_name
		self.odom_topic = "/" + robot_name + "/odom"
		self.laser_topic = "/" + robot_name + "/laser_0"
		self.goal_topic = "/" + robot_name + "/goal"

		""" Localization params """
		self.location = None
		self.map = None

		""" Map as PNG """
		self.data = None
		self.width_x = None
		self.height_y = None
		self.rows = None
		self.resolution = 0.2 # HARD-CODED. INSPECT MAP FILE AND UPDATE

		""" LaserScan"""
		self.scan = None
		self.ang_min = None
		self.ang_max = None
		self.ang_inc = None

		self.initialize_map(map_path)

	def initialize_map(self, map_path):
		r = png.Reader(map_path)
		data = r.asDirect()
		self.width_x = data[0]
		self.height_y = data[1]


		self.info = data[3]
		print(image_2d)

	
	def odom_cbk(self, odom_msg):
		self.location = odom_msg

	def laser_cbk(self, laser_msg):
		self.scan = laser_msg.ranges
		self.ang_min = laser_msg.angle_min
		self.ang_max = laser_msg.angle_max
		self.ang_inc = laser_msg.angle_increment

	def goal_cbk(self, goal_msg):
		self.goal = goal_msg

	def update_map(self, obs_x, obs_y):
		obs_x_map = int(obs_x / self.resolution)
		obs_y_map = int(obs_y / self.resolution)

		new_rows = []
		rownum = 0
		colnum = 0
		
		## Place 3x3 obstacle on map, centered at obs_x_map, obs_y_map
		

	def detect_obstacle(self):
		
		""" retrieve laserscan data for rays directly in front of user """		
		if self.scan == None or self.location == None or self.goal == None:
			print("Not enough valid data to detect obstacle!")
			return 1
		
		mid_range = int(len(self.scan) / 2)
		
		front_ranges = self.scan[mid_range - 5 : mid_range + 5]	
		avg_range = float(sum(front_ranges) / len(front_ranges))

		confidence = 0
		for dist in front_ranges:
			confidence += math.abs(dist - avg_range)


		if confidence >= self.confidence_threshold:
			print("No obstacle detected. Proceeding.")
			return 0

		robot_lin_x = self.location.pose.pose.position.x
		robot_lin_y = self.location.pose.pose.position.y
		robot_ang_z = self.location.pose.pose.orientation.z

		obstacle_loc_x = robot_lin_x + (avg_range * math.cos(robot_ang_z))
		obstacle_loc_y = robot_lin_y + (avg_range * math.sin(robot_ang_z))

		self.update_map(obstacle_loc_x, obstacle_loc_y)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('obstacle_listener', anonymous=True)
    robot = RobotObsDetect("robot0", "/home/kyletucker/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/sparse_obstacles.png")

    rospy.Subscriber(robot.odom_topic, Odometry, robot.odom_cbk)
    rospy.Subscriber(robot.laser_topic, LaserScan, robot.laser_cbk)
    rospy.Subscriber(robot.goal_topic, Pose, robot.goal_cbk)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
