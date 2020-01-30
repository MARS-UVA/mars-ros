
# Compile all packages

> Note: a lot of packages have native dependencies, so it may be cumbersome to make them compile

You'll need jetson-inference, librealsense2 and gRPC installed (see sections below) on your computer before proceeding to `catkin build`. 

To compile all packages, run

```bash
catkin build
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

## Jetson-inference

### On Jetson

Can just follow instructions on https://github.com/hanzhi713/jetson-inference/blob/master/docs/building-repo-2.md

### On Host

> Note: you need to have a cuda-capable Nvidia graphics card

1. Install CUDA-10.0/10.1, you can follow instructions on https://www.tensorflow.org/install/gpu#ubuntu_1804_cuda_101

2. Make sure it is in your path, see https://docs.nvidia.com/cuda/archive/10.1/cuda-installation-guide-linux/index.html#post-installation-actions

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

## Compiling gRPC

First, clone the gRPC repository and initialize submodules

```bash
git clone https://github.com/grpc/grpc
cd grpc
git checkout v1.27.x
git submodule update --init
```

If compiling on jetson, (please skip this if on host)

```bash
cd third_party/boringssl
git checkout master # latest boringssl fixes a compile error on arm64 (about alignment)
cd ..
cd abseil-cpp/
git checkout master # latest abseil fixes a compile error on arm64
cd ../..
```

Finally, invoke CMake and make

```bash
mkdir -p cmake/build
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARE_LIBS=ON -DgRPC_INSTALL=ON
make -j4
sudo make install
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

If have Serialization Error when loading TensorRT models, try removing the engine cache and reload
