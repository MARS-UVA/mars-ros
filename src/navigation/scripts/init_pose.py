#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

# Node initialization
rospy.init_node('init_pose')
pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 1)

# Construct message
init_msg = PoseWithCovarianceStamped()
init_msg.header.frame_id = "map"

# Get initial pose from Gazebo -- in the real world, you can define your robot to be wherever.
# typically midde of static map is origin, so probably the robot start position should be 0,0,0
odom_msg = rospy.wait_for_message('/odom', Odometry) # a one-time subscription
init_msg.pose.pose.position.x = odom_msg.pose.pose.position.x
init_msg.pose.pose.position.y = odom_msg.pose.pose.position.y
# z skipped because it will be 0 in this example
init_msg.pose.pose.orientation.x = odom_msg.pose.pose.orientation.x
init_msg.pose.pose.orientation.y = odom_msg.pose.pose.orientation.y
init_msg.pose.pose.orientation.z = odom_msg.pose.pose.orientation.z
init_msg.pose.pose.orientation.w = odom_msg.pose.pose.orientation.w

# Delay -- optional, here so that user isn't immediately swarmed with mesages
rospy.sleep(1)

# Publish message
rospy.loginfo("setting initial pose")
pub.publish(init_msg)
rospy.loginfo("initial pose is set")