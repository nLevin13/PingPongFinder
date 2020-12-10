#!/usr/bin/env python  
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Pose2D
from worlds.srv import MapAndEndpts, MapAndEndptsResponse

poses = []
curr_target = -1
master_pub = rospy.Publisher('goal_tf_publisher/pong_master/status', String, queue_size=1)

# status = 0: find new path
# status = 1: advance path (don't find new path)
# status = 2: error
def callback(tfcmd_msg):
	global poses, curr_target
	status = tfcmd_msg.cmd
	if status == 0:
		start_pose = tfcmd_msg.start
		end_pose = tfcmd_msg.end
		image_path = tfcmd_msg.image_path
		
		rospy.loginfo('Waiting for path finder service')
		rospy.wait_for_service('path_find')
		rospy.loginfo('Found path finder service')
		try:
			find_path = rospy.ServiceProxy('path_find', MapAndEndpts)
			poses = find_path(start_pose, end_pose, image_path)
			curr_target = 0
			send_success()
		except rospy.ServiceException as e:
	    	print("Service call failed: %s"%e)
	    	send_fail()
	elif status == 1:
		if curr_target < len(poses) - 1:
			curr_target += 1
			send_success()
	else:
		rospy.loginfo('Error ?')
		send_fail()

def tf_service_listener():
	rospy.init_node('tf_service_publisher', anonymous=True)
	rospy.Subsciber('pong_master/goal_tf_publisher/cmd', TFCmd, callback)
	my_spin()

def my_spin():
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	static_transformStamped = geometry_msgs.msg.TransformStamped()

	while not rospy.is_shutdown():
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "map"
        static_transformStamped.child_frame_id = "target"

        pose = poses[curr_target]
        static_transformStamped.transform.translation.x = pose.x
        static_transformStamped.transform.translation.y = pose.y
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, pose.theta)
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
    rospy.rostime.wallsleep(0.5)

def send_success():
	master_pub.publish('Success')

def send_fail():
	master_pub.publish('Failure')

if __name__ == '__main__':
	tf_service_listener()