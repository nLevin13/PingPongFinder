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

#Define the method which contains the main functionality of the node.

class Status(Enum):
	PLAN = 1
	TURN = 2
	DRIVE = 3
	DRIVE_PID = 4
	DETECT = 5
	WAIT = 0

class PongFinder:

	def __init__(self, map_path, drive_routine=1):
		self.drive_routine = drive_routine
		self.map = None
		self.status = status.WAIT	
		self.done = False

		self.goal_poses = [] # array of ordered goal poses.
		self.cur_pose_num = 0 # we currently navigating towards this pose index.
		self.path_len = 0 # len(goal_poses) for logging data?
		
		self.cur_goal = Pose() # Immediate goal pose. self.goal_poses[self.cur_pose_num]
		self.cur_loc = Pose() # Current location, pose converted from odom.

		self.final_goal = Pose() # Final overall goal pose.

	def wait(self):
		# Awaits commands on a rostopic. Perhaps waiting for the "go" button on the cmd line, or in event of error!
		# When exiting wait, should return to following computed path, via
		# self.cur_pose_num and self.goal_poses
		# self.status -> PLAN
		return Status.WAIT

	def plan(self):
		# Plan out new path to goal pose. Update self.poses. Reset self.cur_pose_num to get index of pose
		# Update new final goal, if necessary. 
		# self.status -> TURN or DRIVE_PID, based on self.drive_routine
		return Status.PLAN

	def turn(self):
		# Turn self.cur_goal.angular.z radians. 
		# self.status -> DETECT
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
		# Advance instance variables to next goal pose, or
		# notify if goal is reached.
		# self.status -> TURN, DRIVE_PID, or WAIT


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

			elif self.status == Status.DRIVE_PID:
				rospy.loginfo("Driving to (x=%.2f, y=%.2f) with PID control.", 
					self.cur_goal.linear.x, self.cur_goal.linear.y)
				self.drive_pid()

			elif self.status == Status.DETECT:
				rospy.loginfo("Detecting obstacles directly in front of me!")
				self.detect()
			
			elif self.status == Status.ADVANCE:
				rospy.loginfo("Advancing to pose number (%d)", self.cur_pose_num + 1)
				self.advance()

def controller(robot_frame, target_frame):
  """
  Controls a robot whose position is denoted by robot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - robot_frame: the tf frame of the robot base.
  - target_frame: the tf frame of the desired position.
  """

  ################################### YOUR CODE HERE ##############

  #Create a publisher and a tf buffer, which is primed with a tf listener
  #TODO: replace 'INPUT TOPIC' with the correct name for the ROS topic on which
  # the robot accepts velocity inputs.
  pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)
  
  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz

  K1 = 0.3
  K2 = 1
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      #TODO: Replace 'SOURCE FRAME' and 'TARGET FRAME' with the appropriate TF frame names.
      trans = tfBuffer.lookup_transform('robot0', 'target', rospy.Time())
      # Process trans to get your state error
      # Generate a control command to send to the robot

      control_command = Twist()

      control_command.linear.x = K1*trans.transform.translation.x
      control_command.angular.z = K2*trans.transform.translation.y

      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)



  try:
    controller(sys.argv[1], sys.argv[2])
  except rospy.ROSInterruptException:
    pass

