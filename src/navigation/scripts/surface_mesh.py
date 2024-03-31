#!/usr/bin python3

import rospy
from sensor_msgs.msg import PointCloud2
import subprocess
import open3d as o3d

def pointcloud_callback(msg):
	rospy.loginfo("Pointcloud recived")
	'''pcloud = o3d.geometry.PointCloud()
	pcloud.points = o3d.utility.Vector3dVector(msg.data)
	
	mesh = o3d.geometry.TriangleMesh.create_from
	
	mesh_file = "/tmp/pcloud_mesh.stl"
	o3d.io.write_triangle_mesh(mesh_file, mesh)
	rospy.loginfo("New surface mesh saved to " + mesh_file)'''
	
if __name__ == "__main__":
	#rospy.loginfo("Starting service...")
	rospy.init_node("pointcloud_to_mesh")
	rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloud_callback)
	rospy.spin()
	
