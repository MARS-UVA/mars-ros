#!/bin/bash
rosbag record \
/camera/color/image_raw /camera/color/camera_info \
camera/aligned_depth_to_color/image_raw \
/seg/raw /seg/visual