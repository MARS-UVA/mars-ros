# Run the realsense camera (d435i)
# note: if infra is enabled, its width and height must be the same as the depth width and height
# note: if USB overflow is encountered, probably the system is overloaded.
source ./devel/setup.bash
WIDTH=848
HEIGHT=480
INFRA=false # can disable infra to save bandwidth
roslaunch realsense2_camera rs_camera.launch \
color_width:=$WIDTH color_height:=$HEIGHT \
depth_width:=$WIDTH depth_height:=$HEIGHT \
infra_width:=$WIDTH infra_height:=$HEIGHT \
enable_infra1:=$INFRA enable_infra2:=$INFRA \
unite_imu_method:=copy enable_sync:=true