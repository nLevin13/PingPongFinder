#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys

from soundsim2.msg import AcousticStatus
from enum import Enum
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from pong_driver.msg import DriveCmd
from worlds.msg import TFCmd

#Define the method which contains the main functionality of the node.

class Status(Enum):
	PLAN = 1
	TURN = 2
	DRIVE = 3
	DRIVE_PID = 4
	DETECT = 5
	UPDATE = 6
	ADVANCE = 7
	LISTEN = 8
	WAIT = 0

class PongMaster:

	def __init__(self, map_path, drive_routine):
		# DRIVE ROUTINE: 0 for PID, 1 for TURN + GO w/ OBS
		self.drive_routine = drive_routine
		self.map_path = map_path
		self.status = Status.WAIT	
		self.done = False

		self.start = None
		self.end = None
		self.cur_loc = None # Current location, pose converted from odom.
	
		# DEPRECIATED
		# self.goal_poses = [] # array of ordered goal poses.
		# self.cur_pose_num = None # we currently navigating towards this pose index.
		# self.path_len = 0 # len(goal_poses) for logging data?
		
		# self.cur_goal = None # Immediate goal pose. self.goal_poses[self.cur_pose_num]
		self.dynamic_map_path = '/home/kyletucker/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/sparse_obstacles_dynamic.png'
		self.nav_pub = rospy.Publisher("/pong_master/goal_tf_publisher/cmd", TFCmd, queue_size=1)
		self.cmd_pub = rospy.Publisher("/pong_master/pong_driver/drive_cmd", DriveCmd, queue_size=1)
		self.obs_pub = rospy.Publisher("/pong_master/obstacle_avoid/cmd", String, queue_size=1)
		rospy.sleep(1)


	def wait(self):
		# Awaits commands on a rostopic. Perhaps waiting for the "go" button on the cmd line, or in event of error!
		# When exiting wait, should return to following computed path, via
		# self.cur_pose_num and self.goal_poses
		# self.status -> PLAN
		pass
		# TODO IMPlEMENT WAIT STATE

	def plan(self):
		# Plan out new path to goal pose. Update self.poses. Reset self.cur_pose_num to get index of pose
		# Update new final goal, if necessary. 
		# Nir, your path planning
		# self.status -> TURN or DRIVE_PID, based on self.drive_routine

		assert(self.start != None and self.end != None)

		cmd = TFCmd()
		cmd.start = self.start
		cmd.end = self.end
		cmd.cmd = 0
		cmd.image_path = self.map_path

		# pub = rospy.Publisher('/pong_master/goal_tf_publisher/cmd', TFCmd, queue_size=1)
		pub = self.nav_pub

		rospy.loginfo("Publishing cmd to path planner.")
		pub.publish(cmd)
		rospy.loginfo("Waiting for response...")
		status = rospy.wait_for_message('/goal_tf_publisher/pong_master/status', String)
		if status.data == 'Success':
			rospy.loginfo('Success. Proceeding with drive routine.')
			if self.drive_routine == 0:
				self.status = Status.DRIVE_PID
			elif self.drive_routine == 1:
				self.status = Status.TURN
		else:
			rospy.loginfo('Failure in plan state.')
			self.status = Status.WAIT

	def turn(self):
		# Turn self.cur_goal.angular.z radians. 
		# self.status -> DETECT
		# Assumes appropriate target frame		
		
		# Turn		
		rospy.loginfo('Publishing turn cmd.')	
		self.publish_cmd(1)	

		rospy.loginfo("Awaiting response from pong_driver.")
		status = rospy.wait_for_message('/pong_driver/pong_master/cmd_status', String)

		if status.data == 'Success':
			rospy.loginfo("Success. Status => DETECT")
			# Good, move to detect state, no pose update
			self.status = Status.DRIVE
			# self.status = Status.DETECT

		else:
			# we go into wait state.
			self.status = Status.WAIT
		

	def drive(self):
		# Drive forward self.cur_goal.linear.x meters forward.
		# self.status -> ADVANCE
		rospy.loginfo("Publishing drive cmd.")
		self.publish_cmd(2)
		rospy.loginfo("Waiting for response.")
		status = rospy.wait_for_message('/pong_driver/pong_master/cmd_status', String)
		
		if status.data == 'Success':
			rospy.loginfo("Success. Status => ADVANCE")
			self.status = Status.ADVANCE
		else:
			self.status = Status.WAIT

	def drive_pid(self):
		# Drive to self.cur_goal using unicycle PID
		# self.status -> ADVANCE
		rospy.loginfo("Publishing drive PID cmd.")
		self.publish_cmd(0)
		rospy.loginfo("Waiting for response.")
		status = rospy.wait_for_message('/pong_driver/pong_master/cmd_status', String)

		if status.data == 'Success':
			rospy.loginfo("Success. Status => ADVANCE")
			self.status = Status.ADVANCE
		else:
			status = Status.WAIT
		
	def detect(self):
		# Run obstacle detection and map update routine
		# self.status -> UPDATE or DRIVE
		# CALL DETECTION ROSSERVICE	
		rospy.loginfo("Sending detect command.")
		
		cmd = String()
		cmd.data = 'Detect'
	
		self.obs_pub.publish(cmd)
		rospy.loginfo("Waiting for response.")
		status = rospy.wait_for_message('/obstacle_avoid/pong_master/status', String)
		if status.data == 'Obstacle':
			rospy.loginfo("Obstacle detected. Updating map.")
			self.status = Status.UPDATE
		elif status.data == 'Clear':
			rospy.loginfo("No obstacles. Proceeding to DRIVE.")
			self.status = Status.DRIVE

	def advance(self):
		# Advance instance variables to next goal pose, listening for updates to final goal.
		# Else, notify if goal is reached.
		# self.status -> TURN, DRIVE_PID, or UPDATE
		cmd = TFCmd()
		cmd.cmd = 1
		rospy.loginfo("Sending advance command to goal broadcaster.")
		self.nav_pub.publish(cmd)

		rospy.loginfo("Waiting for response.")
		status = rospy.wait_for_message('/goal_tf_publisher/pong_master/status', String)
		if status.data == 'Success':
			rospy.loginfo("Successful advancement. Proceeding.")
			if self.drive_routine == 0:
				# PID
				self.status = Status.DRIVE_PID
			elif self.drive_routine == 1:
				self.status = Status.TURN
		else:
			self.status = Status.WAIT


	def update(self):
		# We detected an obstacle or updated our final goal pose!
		# Check for updates on both
		# We must plan a new path on the new map, or towards the new goal.
		# self.status -> PLAN
		cmd = TFCmd()
		cmd.cmd = 0
		cmd.start = None
		cmd.end = self.end
		cmd.image_path = self.dynamic_map_path
		
		rospy.loginfo("Sending update request to nav handler.")
		self.nav_pub(cmd)
		rospy.loginfo("Waiting for response.")
		status = rospy.wait_for_message('/goal_tf_publisher/pong_master/status', String)
		if status.data == 'Success':
			rospy.loginfo("Success. Proceeding.")
			if self.drive_routine == 0:
				self.status = Status.DRIVE_PID
			elif self.drive_routine == 1:
				self.status = Status.TURN
		else:
			self.status = Status.WAIT

	def listen(self):
		status = rospy.wait_for_message('location', AcousticStatus)
		self.end = status.goal
		self.status = Status.PLAN	
		

	def drive_loop(self):
		try:
			while not self.done:
				if self.status == Status.LISTEN:
					rospy.loginfo("Listening")
					self.listen()

				elif self.status == Status.WAIT:
					rospy.loginfo("Waiting.")
					self.wait()

				elif self.status == Status.PLAN:
					rospy.loginfo("Planning new path from (x=%.2f, y=%.2f) to (x=%.2f, y=%.2f).", 
						self.start.x, self.start.y, self.end.x, self.end.y)
					self.plan()

				elif self.status == Status.TURN:	
					rospy.loginfo("Turning")
					self.turn()

				elif self.status == Status.DRIVE:
					rospy.loginfo("Driving forward.")
					self.drive()

				elif self.status == Status.DETECT:
					rospy.loginfo("Detecting obstacles directly in front of me!")
					self.detect()
			
				elif self.status == Status.DRIVE_PID:
					rospy.loginfo("Driving to (x=%.2f, y=%.2f) with PID control.", 
						self.end.x, self.end.y)
					self.drive_pid()

				elif self.status == Status.ADVANCE:
					rospy.loginfo("Advancing to next pose.")
					self.advance()

				elif self.status == Status.UPDATE:
					rospy.loginfo("Updating map!")
					self.update()
				rospy.sleep(1)
		except(KeyboardInterrupt):
			rospy.loginfo("Killing master.")


	def publish_cmd(self, drive_command):
		# Assuming that we have the correct goal poses initialized, 
		# publish CMD to robot
		cmd = DriveCmd()		
		cmd.target_frame = 'target'
		cmd.cmd = drive_command
		self.cmd_pub.publish(cmd)
	
	def test(self, x, y, map_path):
		self.end = Pose2D()
		self.start = Pose2D()
		self.start.x = 1
		self.start.y = 2
		self.end.x = x
		self.end.y = y
		self.map_path = map_path
			
		self.status = Status.LISTEN
		self.drive_loop()
		
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell

def test(pongMaster):
	map_path = '/home/kyletucker/ros_workspaces/project/src/PingPongFinder/worlds/src/AStar/testrgb2.png'
	pongMaster.test(7.6, 6.6, map_path)	


if __name__ == '__main__':
	# map_path = '/home/kyletucker/ros_workspaces/project/src/stdr_simulator/stdr_resources/maps/sparse_obstacles.png'	
	map_path = '/home/kyletucker/ros_workspaces/project/src/PingPongFinder/worlds/src/AStar/testrgb2.png'

	pongMaster = PongMaster(map_path, 0)
	rospy.init_node('pong_master', anonymous=True)
	
	test(pongMaster)	

	
	rospy.spin()	





