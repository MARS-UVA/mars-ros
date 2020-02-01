#!/usr/bin/env python
import message_filters
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

bridge = CvBridge()
vis_pub = rospy.Publisher('/seg/visual', Image, queue_size=1)


def callback(rgb_img, depth_img, classes):
    downscale = 2
    max_dist = 2048

    out_width = rgb_img.width // downscale
    out_height = rgb_img.height // downscale
    output = np.zeros((out_height, out_width * 3, 3), dtype="uint8")

    color_img = bridge.imgmsg_to_cv2(rgb_img)
    depth_img = bridge.imgmsg_to_cv2(depth_img)
    mask_np = cv2.resize(bridge.imgmsg_to_cv2(classes), (out_width, out_height), interpolation=cv2.INTER_NEAREST)

    # color the ground with white color.
    # 2 (ground), 3 (road), 4 (sidewalk), 12 (terrain)
    output[:, :out_width, :][
        ((mask_np == 2) | (mask_np == 3) | (mask_np == 4) | (mask_np == 12)) #
    ] = (255, 255, 255)

    cv2.resize(color_img, (out_width, out_height), dst=output[:, out_width:2 * out_width, :])

    # resize depth image
    resized_depth_img = cv2.resize(depth_img, (out_width, out_height), interpolation=cv2.INTER_NEAREST)
    resized_depth_img[resized_depth_img > max_dist] = max_dist
    # visualization
    minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(resized_depth_img)

    # ------ gray scale -----------
    # output[:, 2 * out_width:, :] = cv2.convertScaleAbs(resized_depth_img, alpha=255 / maxVal if maxVal != 0 else 1, beta=0)[:, :, np.newaxis]
    # ------ gray scale -----------
    output[:, 2 * out_width:, :] = cv2.applyColorMap(
        cv2.convertScaleAbs(resized_depth_img, alpha=255.0 / max_dist, beta=0), cv2.COLORMAP_JET
    )

    vis_pub.publish(bridge.cv2_to_imgmsg(cv2.cvtColor(output, cv2.COLOR_BGR2RGB), encoding="bgr8"))
    
def listener():
    rospy.init_node('image_listenor', anonymous=True)
    
    # synchronize three streams
    image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
    depth_sub = message_filters.Subscriber('/camera/aligned_depth_to_color/image_raw', Image)
    cls_sub = message_filters.Subscriber('/seg/raw', Image)

    ts = message_filters.TimeSynchronizer([image_sub, depth_sub, cls_sub], 30)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == "__main__":
    listener()