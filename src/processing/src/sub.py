#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

def callback(data):
    print(data.fields)
    dtype_list = [
        ("x", np.float32),("y", np.float32),("z", np.float32),("unused", np.float32),("rgb", np.float32)
    ]
    print(data.data.__len__(), data.width, data.height)
    cloud_arr = np.fromstring(data.data, dtype_list)
    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_listenor', anonymous=True)

    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()