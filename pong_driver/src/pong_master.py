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

from enum import Enum
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

#Define the method which contains the main functionality of the node.

class Status(Enum):
	PLAN = 1
	TURN = 2
	DRIVE = 3
	DRIVE_PID = 4
	DETECT = 5
	UPDATE = 6
	WAIT = 0

class PongDriver:

	def __init__(self, map_path, drive_routine=1):
		self.drive_routine = drive_routine
		self.map = None
		self.status = status.WAIT	
		self.done = False

		self.goal_poses = [] # array of ordered goal poses.
		self.cur_pose_num = None # we currently navigating towards this pose index.
		self.path_len = 0 # len(goal_poses) for logging data?
		
		self.cur_goal = None # Immediate goal pose. self.goal_poses[self.cur_pose_num]
		self.cur_loc = None # Current location, pose converted from odom.

		self.final_goal = None # Final overall goal pose.

		self.drive_pub = rospy.Publisher("/robot0/drive_cmd")

	def check_goal(self):
		# Checks if we have enough metadata to navigate to the current goal.
		# Returns a bool
		check = False
		if self.cur_goal == None:
			rospy.loginfo("No current goal.")
		elif self.cur_loc == None:
			rospy.loginfo("No current location.")
		elif self.goal_poses == [] or self.cur_pose_num == None:
			rospy.loginfo("No current path.")
		elif self.done:
			rospy.loginfo("Done.")
		else:
			rospy.loginfo("Ready to execute current goal. Goal-Check Passed.")
			return True
		rospy.loginfo("One or more errors. Goal will not be executed.")
		return False


	def odom_cbk(self, msg):
		self.cur_loc = msg.pose.pose

	def wait(self):
		# Awaits commands on a rostopic. Perhaps waiting for the "go" button on the cmd line, or in event of error!
		# When exiting wait, should return to following computed path, via
		# self.cur_pose_num and self.goal_poses
		# self.status -> PLAN

		# TODO IMPlEMENT WAIT STATE
		return Status.WAIT

	def plan(self):
		# Plan out new path to goal pose. Update self.poses. Reset self.cur_pose_num to get index of pose
		# Update new final goal, if necessary. 
		# Nir, your path planning
		# self.status -> TURN or DRIVE_PID, based on self.drive_routine
		return Status.PLAN

	def turn(self):
		# Turn self.cur_goal.angular.z radians. 
		# self.status -> DETECT
				
		# PUBLISH DRIVE CMD
		# AWAIT CONFIRMATION BEFORE PROCEEDING

		


		else:
			# we go into wait state.
			self.status = Status.WAIT
		
		return Status.TURN

	def drive(self):
		# Drive forward self.cur_goal.linear.x meters forward.
		# self.status -> ADVANCE

	def drive_pid(self):
		# Drive to self.cur_goal using unicycle PID
		# self.status -> ADVANCE

	def detect(self):
		# Run obstacle detection and map update routine
		# self.status -> UPDATE or DRIVE
				
	def advance(self):
		# Advance instance variables to next goal pose, listening for updates to final goal.
		# Else, notify if goal is reached.
		# self.status -> TURN, DRIVE_PID, or UPDATE

	def update(self):
		# We detected an obstacle or updated our final goal pose!
		# Check for updates on both
		# We must plan a new path on the new map, or towards the new goal.
		# self.status -> PLAN
		

	def drive_loop(self):

		while not self.done:

			if self.status == Status.WAIT:
				rospy.loginfo("Currently waiting at path pose (%d) out of (%d).", self.cur_pose_num, self.path_len)
				self.wait()

			elif self.status == Status.PLAN:
				rospy.loginfo("Planning new path from (x=%.2f, y=%.2f) to (x=%.2f, y=%.2f).",
					self.cur_loc.position.x, self.cur_loc.position.y, 
						self.final_goal.position.x, self.final_goal.position.y)
				self.plan()

			elif self.status == Status.TURN:	
				rospy.loginfo("Turning (%.2f) radians.", self.cur_cmd.angular.z)
				self.turn()

			elif self.status == Status.DRIVE:
				rospy.loginfo("Driving (%.2f) meters forward.", self.cur_cmd.linear.x)
				self.drive()

			elif self.status == Status.DETECT:
				rospy.loginfo("Detecting obstacles directly in front of me!")
				self.detect()
			
			elif self.status == Status.DRIVE_PID:
				rospy.loginfo("Driving to (x=%.2f, y=%.2f) with PID control.", 
					self.cur_goal.linear.x, self.cur_goal.linear.y)
				self.drive_pid()

			elif self.status == Status.ADVANCE:
				rospy.loginfo("Advancing to pose number (%d)", self.cur_pose_num + 1)
				self.advance()

			elif self.status == Status.UPDATE:
				rospy.loginfo("Updating map!")
				self.update()

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell

if __name__ == '__main__':
	
	pongDriver = PongDriver(map_path)
	rospy.init_node('pong_driver', anyonymous=True)
	rospy.Subscriber("/robot0/odom", Odometry, pongDriver.odom_cbk)
	rospy.spin()	





