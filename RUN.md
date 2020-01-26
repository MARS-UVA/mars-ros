Run the realsense camera (d435i)

```bash
roslaunch realsense2_camera rs_camera.launch color_width:=1280 color_height:=720 color_fps:=30 depth_width:=1280 depth_height:=720 depth_fps:=15 align_depth:=true enable_sync:=true
```