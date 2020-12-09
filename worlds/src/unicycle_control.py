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
import numpy as np
from math import fabs, sqrt

from geometry_msgs.msg import Twist

#Define the method which contains the main functionality of the node.
def controller(robot_frame, target_frame, target_frame2):
  """
  Controls a robot whose position is denoted by robot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - robot_frame: the tf frame of the robot base.
  - target_frame: the tf frame of the desired position.
  """
  print(robot_frame, target_frame, target_frame2)
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

  K1 = 0.4
  K2 = 1
  error = float('inf')
  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown() and fabs(error) > 0.5:
    try:
      #TODO: Replace 'SOURCE FRAME' and 'TARGET FRAME' with the appropriate TF frame names.
      trans = tfBuffer.lookup_transform(robot_frame, target_frame, rospy.Time())
      # Process trans to get your state error
      # Generate a control command to send to the robot
      res_trans = trans.transform.translation
      control_comm = np.matmul(np.array([[K1, 0], [0, K2]]),\
        np.array([[res_trans.x], [res_trans.y]]))
      control_command = Twist()
      control_command.linear.x = control_comm[0]
      control_command.angular.z = control_comm[1]
      error = sqrt(res_trans.x**2 + res_trans.y**2)
      print(error)
      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print("exception")
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

  error = float('inf')
  while not rospy.is_shutdown() and fabs(error) > 0.1:
    try:
      #TODO: Replace 'SOURCE FRAME' and 'TARGET FRAME' with the appropriate TF frame names.
      trans = tfBuffer.lookup_transform(robot_frame, target_frame2, rospy.Time())
      # Process trans to get your state error
      # Generate a control command to send to the robot
      res_trans = trans.transform.translation
      control_comm = np.matmul(np.array([[K1, 0], [0, K2]]),\
        np.array([[res_trans.x], [res_trans.y]]))
      control_command = Twist()
      control_command.linear.x = control_comm[0]
      control_command.angular.z = control_comm[1]
      error = sqrt(res_trans.x**2 + res_trans.y**2)
      print(error)
      #################################### end your code ###############

      pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      print("exception")
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

  control_command = Twist()
  control_command.linear.x = 0
  control_command.angular.z = 0
  pub.publish(control_command)


      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  print("hi")
  rospy.init_node('turtlebot_controller', anonymous=True)

  try:
    controller(sys.argv[1], sys.argv[2], sys.argv[3])
  except rospy.ROSInterruptException:
    pass

