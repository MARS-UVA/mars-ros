# MARS - ROS
ROS packages for the Mechatronics and Robotics Society at the University of Virginia.

These packages are installed on the team's robot.

## Package Descriptions
- actions: handles motor routines like raising bucket ladder, digging, etc. 
- hero_board: communication between the hero board and the jetson
- navigation: used for autonomous navigation

## Usage on Robot
To run ROS nodes on the robot and connect to them from the control station, navigate to the root directory of this project and run 
`./start1.sh` in one terminal and `./start2.sh` in another.

## Direct Usage: Launch Files
Once these ROS packages are built and sourced, any nodes in them can be run like any other ROS node. However, we recommend using the code via the launch files we have created.

### Main launch file
The `navigation/launch/malvi_config.launch` launch file is used to start the robot. Refer to the launch file for possible parameters.

```bash
roslaunch navigation malvi_config.launch
```

### Additional launch files
We have provided several other launch files for running other ROS nodes outside the typical robot-control use case.

#### lidar-and-tracking-branch
Runs nodes to start an RP LiDAR and Intel Realsense Tracking Camera T265 that will be mounted on the robot.
```bash
roslaunch navigation lidar-and-tracking.launch
```

## Containerized Usage: Docker 
For a consistent production environment, we deploy our ROS code in Docker containers. These containers install ROS, set up required packages, and run one of the aforementioned launch files.

### Main image
The main Docker image (which can be built using `Dockerfile`) runs our main launch file, `navigation/launch/malvi_config.launch`.\
Build:
```bash
docker build . -t mars-ros
```
Run:
```bash
docker run --rm -it --network host mars-ros
```
### Lidar and tracking camera image
Runs the `navigation/launch/lidar-and-tracking.launch` launch file.\
Build:
```bash
docker build . -t mars-lidar-and-tracking -f Dockerfile.lidar_and_tracking
```
Run:
```bash
docker run --rm -it --network host --volume=/dev:/dev --privileged mars-lidar-and-tracking
```
The USB devices for the camera and LiDAR must be passed through to the container. Due to a quirk of the tracking camera where it is only recognized after its ROS node is started, we must mount /dev on the host to /dev in the container.

## Cloning and Pulling
Because this repository uses submodules, care must be taken when pulling/pushing. To clone this repository, you need to add the `--recursive` flag:

```bash
git clone --recursive https://github.com/MARS-UVA/mars-ros.git
```

If you forgot the `--recursive` flag, you need to run `submodule init` manually after cloning

```bash
git submodule update --init
```

To pull changes from the remote, you need to run two commands

```bash
git pull
git submodule update --init
```

## Note for Development
When developing for specific packages, you do not need to compile all other packages. You can specify the name of the packages you want to build as arguments. 

### VSCode Remote Development on Jetson
[VSCode remote development](https://code.visualstudio.com/docs/remote/ssh) works well. However, for the Python/C++ language extension to work, you need to tweak some settings.

#### Python:
Microsoft Python Language Server does not support arm64. Therefore, you need to add `"python.jediEnabled": true` to the workspace settings

#### C++:
Microsoft C++ extension does not support arm64. You need to install the `vscode-clangd` plugin from llvm instead. Follow the plugin README to get started?

## Compiling All Packages
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