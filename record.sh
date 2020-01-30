#!/bin/bash
rosbag record /camera/color/image_raw /camera/color/camera_info \
/camera/depth/image_rect_raw /camera/depth/camera_info \
/camera/extrinsics/depth_to_color /seg /seg/visual