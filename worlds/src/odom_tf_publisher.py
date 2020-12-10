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

    t.header.stamp = msg.header.stamp

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    

    t.header.frame_id = "map_static"
    t.child_frame_id = "odom"

    br.sendTransform(t)
    print("Transform sent")
    print(t.transform)

    rospy.sleep(.1)
    

if __name__ == '__main__':
    rospy.init_node('odomtf')
    rospy.Subscriber('robot0/odom', Odometry, publish_tf, queue_size=1)
    rospy.spin()
