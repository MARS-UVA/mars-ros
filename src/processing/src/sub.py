#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

rgb_pc_list = [
    ("x", np.float32), ("y", np.float32), ("z", np.float32), ("unused", np.float32), ("rgb", np.float32)
]
pc_list = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("unused", np.float32)]


def callback(data):
    cloud_arr = np.fromstring(data.data, "float32").reshape(data.height, data.width, 4)
    arr = cloud_arr[~np.isnan(cloud_arr)]
    print(arr[arr != 0])
    # print(cloud_arr)


def listener():
    rospy.init_node('pc_listenor', anonymous=True)
    rospy.Subscriber("/camera/points", PointCloud2, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
