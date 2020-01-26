# Run the realsense camera (d435i)
source ./devel/setup.zsh
roslaunch realsense2_camera rs_camera.launch color_width:=848 color_height:=480 color_fps:=30 depth_width:=848 depth_height:=480 depth_fps:=30 enable_sync:=true