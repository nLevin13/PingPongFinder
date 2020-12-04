#!/usr/bin/env python
import rospy, pypng
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs import LaserScan
from geometry_msgs import Pose


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

		""" LaserScan"""
		self.scan = None
		self.ang_min = None
		self.ang_max = None
		self.ang_inc = None

		self.initialize_map(map_path)

	def initialize_map():
		r = png.Reader(map_path)
		data = r.read()
		self.width_x = data[0]
		self.height_y = data[1]
		self.rows = data[2]
		self.info = data[3]

	
	def odom_cbk(self, odom_msg):
		self.location = odom_msg

	def laser_cbk(self, laser_msg):
		self.scan = laser_msg.ranges
		self.ang_min = laser_msg.angle_min
		self.ang_max = laser_msg.angle_max
		self.ang_inc = laser_msg.angle_increment

	def goal_cbk(self, goal_msg):
		self.goal = goal_msg

	def detect_obstacle(self):
		
		""" retrieve laserscan data for rays directly in front of user """		
		if self.scan == None or self.location == None or self.goal == None:
			print("Not enough valid data to detect obstacle!")
			return 0
		
		mid_range = int(len(self.scan) / 2)
		
		front_ranges = self.scan[mid_range - 5 : mid_range + 5]	

		



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def odom_callback(odom):
	

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('obstacle_listener', anonymous=True)

	robot = RoboObsDetect("robot0", "~/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/sparse_obstacles.png")

    rospy.Subscriber(self.odom_topic, Odometry, robot.odom_cbk)
    rospy.Subscriber(self.laser_topic, LaserScan, robot.laser_cbk)
    rospy.Subscriber(self.goal_topic, Pose, robot.goal_cbk)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
