# MARS - ROS
ROS packages for the MARS club

## Package Description
- actions: handles motor routines like raising bucket ladder, digging, etc. 
- camera_streamer: 
- hero_board: communication between the hero board and the jetson
- navigation: used for autonomous navigation
- ~~processing: ROS nodes for processing sensors data~~
- rpc-server: RPC server for communication with the remote control station (the laptop)
- ~~segmentation: uses machine learning to differntiate ground and environment~~

## Launch files
Under navigation/launch, the file malvi_config.launch is the main launch file that is used to start the robot. Refer to the launch file for possible parameters. The default parameters assume all aspects of the robot should be run. 

```bash
roslaunch navigation malvi_config.launch
```

## Cloning and Pulling
Because this repository uses submodules, care must be taken when pulling/pushing. To clone this repository, you need to add the `--recursive` flag:

```bash
git clone --recursive https://github.com/MARS-UVA/mars-ros.git
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


## Note for development
When developing for specific packages, you do not need to compile all other packages. You can specify the name of the packages you want to build as arguments. 

### VSCode Remote Development on Jetson
[VSCode remote development](https://code.visualstudio.com/docs/remote/ssh) works well. However, for the Python/C++ language extension to work, you need to tweak some settings.

#### Python:
Microsoft Python Language Server does not support arm64. Therefore, you need to add `"python.jediEnabled": true` to the workspace settings

#### C++:
Microsoft C++ extension does not support arm64. You need to install the `vscode-clangd` plugin from llvm instead. Follow the plugin README to get started?

## Compile all packages
If you need to compile all packages, refer to the [compile guide](./CompileGuide.md)

### Managing package dependencies
All performed in `~/catkin_ws/`
1. Generate a dependency list with [rosinstall_generator](https://wiki.ros.org/rosinstall_generator): `rosinstall_generator $(cat ~/mars-ros/packages.txt) --rosdistro noetic --deps --wet-only --tar > wet1.rosinstall`
2. Install ROS dependencies with [wstool](https://wiki.ros.org/wstool)
    - If it's the first time running wstool, use `wstool init -j4 -t src wet1.rosinstall`
    - If you're updating the list of packages, use `wstool merge -t src wetX.rosinstall` then `wstool update -j4 -t src`
3. Install system dependencies: `rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=ubuntu:bionic`
4. Build packages: `sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j4 -DPYTHON_EXECUTABLE=/usr/bin/python3`
    - To build specific packages, add on `--pkg package1 package2`