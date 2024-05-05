#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import pcl
from sensor_msgs import point_cloud2 as pc2
import numpy as np

def pcl_callback(msg):
	cloud = pcl.PointCloud()
	points = np.array(list(pc2.read_points(msg)))
	cloud.from_array(points)
	
	for p in points:
		print("x:", p[0], "y:", p[1], "z:", p[2])
		
if __name__ == "__main__":
	rospy.init_node("realsense_pcl")
	rospy.Subscriber('/camera/depth/color/points', PointCloud2, pcl_callback)
	rospy.spin()
	
	
