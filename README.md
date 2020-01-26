# MARS - ROS

ROS packages for the MARS club

## Cloning and Pulling

> Because this repository uses submodules, care must be taken when pulling/pushing 

To clone this repository, you need to add the `--recursive` flag

```bash
git clone --recursive https://github.com/hanzh713/mars-ros
```

If you forgot the `--recursive` flag, you need to run submodule init manually after cloning

```bash
git submodule update --init
```

To pull changes from the remote, you need to run two commands

```bash
git pull
git submodule update --init
```

For more information about git submodules, refer to the git handbook: https://git-scm.com/book/en/v2/Git-Tools-Submodules

## Note for development

When developing for specific packages, you do not need to compile all other packages. You can use the `-DCATKIN_BLACKLIST_PACKAGES` option to blacklist packages you don't want to compile.
For example,

```bash
catkin_make -DCATKIN_BLACKLIST_PACKAGES="apriltag;apriltag_ros;segmentation"
```

will ignore the `apriltag`, `apriltag_ros` and `segmentation` packages. 

## Compile all packages

> Note: a lot of packages have native dependencies, so it may be cumbersome to make them compile

You'll need jetson-inference and librealsense2 installed (see sections below) on your computer before proceed. Then, you can just run

```bash
catkin_make_isolated
```

### Librealsense2

On host, can just install .deb packages from https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

On jetson, need to build from source. Run buildLibrealsense.sh from https://github.com/JetsonHacksNano/installLibrealsense

### jetson-inference on jetson

Can just follow instructions on https://github.com/hanzhi713/jetson-inference/blob/master/docs/building-repo-2.md

### jetson-inference on host

> Note: you need to have a cuda-capable Nvidia graphics card

1. Install CUDA-10.0/10.1, you can follow instructions on https://www.tensorflow.org/install/gpu#ubuntu_1804_cuda_101

2. Make sure it is in your path, see

https://docs.nvidia.com/cuda/archive/10.1/cuda-installation-guide-linux/index.html#post-installation-actions

3. Download TensorRT 6.0.1 (tar ball), and copy headers and shared libraries to system include path

```bash
cd TensorRT-6.x.x.x
sudo cp -r include/* /usr/include/x86_64-linux-gnu/
sudo cp -r lib/* /usr/lib/x86_64-linux-gnu/
```

> Note: do not use tensorrt >= 7, otherwise you'll encounter an implicit batch dimension error

4. Clone the repository

```bash
git clone --recursive https://github.com/hanzhi713/jetson-inference
```

5. Modify the CMakeLists.txt

Open CMakeLists.txt and replace `aarch64-linux-gnu` with `x86_64-linux-gnu`.

Additionally, find the CUDA_NVCC_FLAGS and add `-gencode arch=compute_xx,code=sm_xx` where `xx` is your graphic cards' compute compatibility

6. Build and install

```bash
cd jetson-inference
mkdir build
cd build
cmake ../
make
sudo make install
sudo ldconfig
```

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