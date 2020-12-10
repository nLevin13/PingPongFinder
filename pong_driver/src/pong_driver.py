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

import pong_driver.msgs import DriveCmd
from tf import transformations
from nav_msgs.msg import Odometry
from enum import Enum
from geometry_msgs.msg import Twist

THRESHOLD = .25
THRESHOLD_RAD = .1

def turn_naiive(target_frame):
	pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=3)
    tfBuffer = tf2_ros.Buffer()
    tfListender = tf2_ros.TransformListender(tfBuffer)
    r = rospy.Rate(10)

	done = False
	while not done or not rospy.is_shutdown():
		try:
			transform = tfBuffer.lookup_transform('odom', target_frame, rospy.Time())
			if within_threshold(transform):
				done = True
                rospy.loginfo("Turning goal pose achieved. Sending success message to PongMaster.")
				send_success()
                cmd.linear.x = 0
                cmd.angular.z = 0
            else:
                cmd.angular.z = K2 * transform.transform.translation.y

            pub.publish(cmd)


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF Exception encountered. Sending Failure.")
            send_failure()
            done = True
            pass


def drive_naiive(target_frame):
	pub = rospy.Publisher('robot0/cmd_vel', Twist, queue_size=3)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListender(tfBuffer)
    r = rospy.Rate(10)

	done = False
	while not done or not rospy.is_shutdown():
		try:
			transform = tfBuffer.lookup_transform('odom', target_frame, rospy.Time())
			if turn_within_threshold(transform)
				
				rospy.loginfo("Driving goal pose achieved. Sending success message to PongMaster.")
				send_success()
                cmd.linear.x = 0
                cmd.angular.z = 0
                done = True
            else:
                cmd.linear.x = K1 * transform.transform.translation.x

            pub.publish(cmd)

            if

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF Exception encountered. Sending Failure.")
            send_failure()
            done = True
            pass



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
			transform = tfBuffer.lookup_transform('odom', target_frame, rospy.Time())
			cmd = Twist()

			if within_threshold(transform):
				rospy.loginfo("Goal pose achieved. Sending success message to PongMaster.")
				send_success()
				cmd.linear.x = 0
				cmd.angular.z = 0
				done = True
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


def drive_cmd_callback(drive_cmd):
	# drive_cmd.cmd => 0 = PID, 1 = TURN, 2 = FWD
	if drive_cmd.cmd == 0:
		drive_PID(drive_cmd.target_frame)
	elif drive_cmd.cmd == 1:
		turn_naiive(drive_cmd.target_frame)
	elif drive_cmd.cmd == 2:
		drive_naiive(drive_cmd.target_frame)
	return


def obstacle_avoid_driver(target_frame):
	turn_naiive(tf_frame)

def turn_within_threshold(transform):
	rot = transform.transform.rotation
	euler = transformations.euler_from_quaterion(rot)
	return euler[2] < THRESHOLD_RAD

def within_threshold(transform):
	error = (transform.transform.translation.x ** 2) + (transform.transform.translation.y ** 2)
	return sqrt(error) < THRESHOLD

def send_failure():
	pub = rospy.Publisher('/pong_driver/robot0/cmd_status', String, queue_size=1)
	pub.publish('Failure')

def send_success():
	pub = rospy.Publisher('/pong_driver/robot0/cmd_status', String, queue_size=1)
	pub.publish('Success')

if __name__ == '__main__':
	rospy.init_node('pong_driver', anonymous=True)
	rospy.Subscriber('/robot0/odom', Odometry, odom_cbk)
	rospy.Subscriber('/pong_master/robot0/drive_cmd', DriveCmd, drive_cmd_callback)

	rospy.spin()
