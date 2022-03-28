#!/bin/sh

echo Enter this device\'s IP address: \(without http:// or port\)
read IP

source /opt/ros/melodic/setup.bash
source devel_isolated/setup.bash

export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP
unset ROS_HOSTNAME # Setting both ROS_IP and ROS_HOSTNAME isn't necessary and it might cause problems if they're different

echo Done.
