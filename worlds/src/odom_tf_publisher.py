#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg
from nav_msgs.msg import Odometry

def publish_tf(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map_static"
    t.child_frame_id = "odom"

    br.sendTransform(t)
    print("Transform sent")


    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0

    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "robot0"

    br.sendTransform(t)

    rospy.sleep(.1)
    

if __name__ == '__main__':
    rospy.init_node('odomtf')
    rospy.Subscriber('robot0/odom', Odometry, publish_tf, queue_size=1)
    rospy.spin()
