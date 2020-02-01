#!/usr/bin/env python
from __future__ import print_function
import rospy
import message_filters
from sensor_msgs.msg import PointCloud2, PointField, Image
from cv_bridge import CvBridge
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import Queue

bridge = CvBridge()

rgb_pc_list = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("unused", np.float32), ("rgb", np.float32)]
pc_list = [("x", np.float32), ("y", np.float32), ("z", np.float32), ("unused", np.float32)]

queue = Queue.Queue(30)


def callback(pc, seg):
    pc_arr = np.fromstring(pc.data, "float32").reshape(pc.height, pc.width, 4)
    seg_arr = bridge.imgmsg_to_cv2(seg)

    lower_half = np.zeros(seg_arr.shape, "bool")
    lower_half[(seg.height * 2) / 3:, seg.width / 3:(seg.width * 2) / 3] = 1
    mask = (lower_half & ((seg_arr == 2) | (seg_arr == 3) | (seg_arr == 4) | (seg_arr == 12)))

    ground = pc_arr[mask]
    ground = ground[np.count_nonzero(np.isnan(ground), axis=1) == 0]

    A = np.c_[ground[:, 0], ground[:, 1], np.ones(ground.shape[0])]
    C, _, _, _ = np.linalg.lstsq(A, ground[:, 2])    # coefficients

    t = time.clock()
    queue.put((C, ground))
    print(C, time.clock() - t)


def listener():
    rospy.init_node('pc_listenor', anonymous=True)

    pc_sub = message_filters.Subscriber('/camera/points', PointCloud2)
    seg_sub = message_filters.Subscriber('/seg/raw', Image)

    ts = message_filters.TimeSynchronizer([pc_sub, seg_sub], 30)
    ts.registerCallback(callback)

    X, Y = np.meshgrid(np.arange(-3.0, 3.0, 0.5), np.arange(-3.0, 3.0, 0.5))

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    plt.show(block=False)

    while not rospy.is_shutdown():
        try:
            C, ground = queue.get_nowait()
        except Queue.Empty:
            continue
            
        Z = C[0]*X + C[1]*Y + C[2]

        ax.clear()
        ax.plot_surface(X, Y, Z, rstride=1, cstride=1, alpha=0.2)
        ax.scatter(ground[:100, 0], ground[:100, 1], ground[:100, 2], c='r', s=10)

        fig.canvas.draw_idle()
        fig.canvas.start_event_loop(0.001)
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # ax.set_zlabel('Z')
        # ax.axis('tight')
        # plt.pause(0.01)


if __name__ == '__main__':
    listener()
