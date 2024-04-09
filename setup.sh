#!/bin/bash
USERNAME="vboxuser"

# source ROS
source /opt/ros/noetic/setup.bash

# source our catkin workspace
# there are two possible ways that the ros packages may have been compiled; this block lets us source in either case
if [ -f "devel/setup.bash" ]; then
    source devel/setup.bash
elif [ -f "devel_isolated/setup.bash" ]; then
    source devel_isolated/setup.bash
else
    echo "Could not find a ROS setup.bash file! ROS commands may not work."
fi

# if the file permissions to edit the connection to the HERO board are incorrect
if [[ -z $(getent group | grep $USERNAME | grep -o dialout) ]]; then
echo "File permissions incorrect; applying fix. Please log out and log back in for the fix to be applied."
    sudo usermod -a -G dialout $USERNAME # add the user to the group that has access
fi

# if the USB connection to the HERO board cannot be found
if [ ! -e "/dev/ttyUSB0" ]; then
    echo "Could not find device /dev/ttyUSB0 (HERO board)!"
fi

# if some ros dependencies are not installed
if ! rosdep check actions | grep -q 'satisfied'; then
    rosdep install actions
fi
if ! rosdep check navigation | grep -q 'satisfied'; then
  rosdep install navigation
fi
if ! rosdep check hero_board | grep -q 'satisfied'; then
  rosdep install hero_board
fi

echo "Done."
