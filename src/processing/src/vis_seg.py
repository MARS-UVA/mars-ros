#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()
vis_pub = rospy.Publisher('/seg/visual', Image, queue_size=1)

def callback(data):
    img = np.zeros((data.height, data.width), "uint8")
    mask = bridge.imgmsg_to_cv2(data)
    img[((mask == 2) | (mask == 3) | (mask == 4) | (mask == 12))] = 255
    vis_pub.publish(bridge.cv2_to_imgmsg(img))
    
def listener():
    rospy.init_node('image_listenor', anonymous=True)
    rospy.Subscriber("/seg", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()