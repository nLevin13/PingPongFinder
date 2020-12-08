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

from nav_msgs.msg import Odometry
from enum import Enum
from geometry_msgs.msg import Twist

THRESHOLD = .25

def drive_cmd_callback(tf_frame):
	drive_PID(tf_frame)

def within_threshold(transform):
	error = (transform.transform.translation.x ** 2) + (transform.transform.translation.y ** 2)
	return sqrt(error) < THRESHOLD


def send_failure():
	pub = rospy.Publisher('/pong_driver/robot0/cmd_status', String, queue_size=1)
	pub.publish('Failure')

def send_success():
	pub = rospy.Publisher('/pong_driver/robot0/cmd_status', String, queue_size=1)
	pub.publish('Success')

def drive_PID(target_frame):
	pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=3)
	tfBuffer = tf2_ros.Buffer()
	tfListender = tf2_ros.TransformListender(tfBuffer)
	r = rospy.Rate(10)
	K1 = 0.3
	K2 = 1.0

	done = False
	while not done or not rospy.is_shutdown():
		try:
			transform = tfBuffer.lookup_transform('robot0', 'odom', rospy.Time())
			cmd = Twist()

			if within_threshold(transform):
				rospy.loginfo("Goal pose achieved. Sending success message to PongMaster.")
				send_success()
				cmd.linear.x = 0
				cmd.angular.z = 0

			else:
				cmd.linear.x = K1 * transform.transform.translation.x
				cmd.angular.z = K2 * transform.transform.translation.y
			
			pub.publish(cmd)

			if 

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.loginfo("TF Exception encountered. Sending Failure.")
			send_failure()
			done = True
			pass

if __name__ == '__main__':
	rospy.init_node('pong_driver', anonymous=True)
	rospy.Subscriber('/robot0/odom', Odometry, odom_cbk)
	rospy.Subscriber('/pong_master/robot0/drive_cmd', String, drive_cmd_callback)

	pid_loop()
	rospy.spin()
      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
