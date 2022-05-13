#!/bin/bash

echo "Enter this device's IP address (laptop): (without http:// or port)"
echo "    (if running this in a VM, enter the host computer's IP)"
read IP

source /opt/ros/melodic/setup.bash

if [ -f "devel_isolated/setup.bash" ]; then
    # echo "Sourcing devel_isolated/setup.bash"
    source devel_isolated/setup.bash
elif [ -f "devel/setup.bash" ]; then
    # echo "Sourcing devel/setup.bash"
    source devel/setup.bash
else
    echo "Could not find a ROS setup.bash file! ROS commands may not work."
fi

if [ -f "/dev/ttyUSB0" ]; then
    chmod 666 /dev/ttyUSB0 # requires sudo but will prompt the user for password
else
    echo "Could not find device /dev/ttyUSB0 (HERO board)! File permissions may be wrong and it may not be found by ROS."
fi


export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP
unset ROS_HOSTNAME # Setting both ROS_IP and ROS_HOSTNAME isn't necessary and it might cause problems if they're different

echo Done.
