#!/bin/bash

echo "Enter this device's IP address (Jetson): (without http:// or port)"
echo "    (if running this in a VM, enter the guest computer's IP)"
read IP

source /opt/ros/melodic/setup.bash

if [ -f "devel_isolated/setup.bash" ]; then
    echo "Sourcing devel_isolated/setup.bash..."
    source devel_isolated/setup.bash
elif [ -f "devel/setup.bash" ]; then
    echo "Sourcing devel/setup.bash..."
    source devel/setup.bash
else
    echo "Could not find a ROS setup.bash file! ROS commands may not work."
fi

if [ -e "/dev/ttyUSB0" ]; then
    echo "Doing chmod on /dev/ttyUSB0..."
    sudo chmod 666 /dev/ttyUSB0 # will prompt the user for password, don't have to run the whole script as sudo
else
    echo "Could not find device /dev/ttyUSB0 (HERO board)! File permissions may be wrong and it may not be found by ROS."
fi


export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP
unset ROS_HOSTNAME # Setting both ROS_IP and ROS_HOSTNAME isn't necessary and it might cause problems if they're different

echo Done.
