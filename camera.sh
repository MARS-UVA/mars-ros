# Run the realsense camera (d435i)
source ./devel/setup.bash
# note: if infra is enabled, its width and height must be the same as the depth width and height 
roslaunch realsense2_camera rs_camera.launch \
color_width:=848 color_height:=480 \
depth_width:=848 depth_height:=480 \
enable_infra1:=false enable_infra2:=false enable_sync:=true \
unite_imu_method:=copy