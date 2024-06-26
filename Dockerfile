# Use an official ROS base image
FROM ros:noetic

# Install git
RUN apt update && apt install -y git

# Clone the mars-ros repository
RUN git clone https://github.com/MARS-UVA/mars-ros.git mars-ros
WORKDIR /mars-ros

RUN git pull
RUN git checkout origin/change-hero-feedback

# Install python3-serial using pip3
RUN apt install -y python3-pip && \
    pip3 install pyserial

# Install ROS development tools
RUN apt install -y python3-catkin-tools

# Update rosdep and install ROS dependencies from mars-ros/src
RUN rosdep update
RUN rosdep install -y --from-paths src --ignore-src --rosdistro noetic

# Build ROS packages
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source the ROS setup.bash file every time a container is started
RUN sed --in-place --expression \
	'$isource "/mars-ros/devel/setup.bash"' \
	/ros_entrypoint.sh

# Start the mars-ros main launch file when the container runs
CMD [ "/bin/bash", "-c", "roslaunch navigation malvi_config.launch hero:=true apriltags:=false" ]
