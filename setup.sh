#!/bin/bash

echo "Enter this device's IP address (Jetson): (without http:// or port)"
echo "    (if running the rpc server in a VM, enter the guest OS's IP)"
echo "    (find IP using in Linux 'hostname -I')"
read IP

source /opt/ros/noetic/setup.bash

if [ -f "devel/setup.bash" ]; then
    # echo "Sourcing devel/setup.bash"
    source devel/setup.bash
elif [ -f "devel_isolated/setup.bash" ]; then
    # echo "Sourcing devel_isolated/setup.bash"
    source devel_isolated/setup.bash
else
    echo "Could not find a ROS setup.bash file! ROS commands may not work."
fi

if [ -e "/dev/ttyUSB0" ]; then
    sudo chmod 666 /dev/ttyUSB0 # requires sudo but will prompt the user for password
else
    echo "Could not find device /dev/ttyUSB0 (HERO board)! File permissions may be wrong and it probably won't be found by ROS."
fi


export ROS_MASTER_URI=http://$IP:11311
export ROS_IP=$IP
unset ROS_HOSTNAME # Setting both ROS_IP and ROS_HOSTNAME isn't necessary and it might cause problems if they're different

echo "Done."
