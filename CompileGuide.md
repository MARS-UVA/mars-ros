
# Compile all packages

You can compile all the packages and dependencies for this project in one go with catkin. However, you'll need librealsense2 installed (see section below) on your computer before proceeding. 

To compile all packages, run

```bash
catkin_make
```

OR

```bash
catkin_make_isolated
```

## Librealsense2

### On Host

With Ubuntu LTS kernel < 5.0, can just install .deb packages from https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

With the latest Ubuntu 18.04.3 kernels (>=5.0), need to build from source and bypass libuvc. Follow the instructions available at https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md, but skip kernel patches (skip `./scripts/patch-realsense-ubuntu-lts.sh`) and add `-DFORCE_RSUSB_BACKEND=true` flag when invoking cmake.

```bash
cmake ../ -DFORCE_RSUSB_BACKEND=true -DCMAKE_BUILD_TYPE=release -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=true
```

### On Jetson

Need to build from source. Run buildLibrealsense.sh from https://github.com/JetsonHacksNano/installLibrealsense

---

### Misc

If you have OpenCV4 but encounter an error: OpenCV not found, then try

```bash
cd /usr/include
sudo ln -s opencv4 opencv
```

ddnynamic reconfigure:

```bash
sudo apt install ros-melodic-ddynamic-reconfigure
```

Libopencv-photo.so.3.2.0 not found:

```bash
sudo apt install libopencv3.2
```

If have Serialization Error when loading TensorRT models, try removing the engine cache and reload
