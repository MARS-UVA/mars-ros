#!/bin/zsh

<<instructions
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot). This should be run AFTER start1.sh.

Instructions: source start2.sh 
					or 
			  source start2.sh <jetson's IP address>
instructions

# Define the IP address of the Jetson
JETSON_IP=${1:-"192.168.0.102"}

# Set environment variables that will allow the control station to find the Jetson on the network
cd
control_station_IP=$(hostname -I | awk '{print $1}')
# control_station_IP=${1:-"192.168.0.101"}
nvidia_count=`grep nano /etc/hosts -c`

if [ $nvidia_count -gt 0 ]
then
	echo "mars2324" | sudo -S sed -i "s/.* nano/$JETSON_IP nano/g" /etc/hosts
else
	echo "mars2324" | sudo -S sed -i "/The following*/i $control_station_IP nano\n" /etc/hosts
fi

# Start a rosbridge server to connect the control station to the ROS nodes on the Jetson
gnome-terminal -- bash -c "source mars/mars-ros/devel/setup.bash; export ROS_MASTER_URI=http://nano:11311; export ROS_IP=$control_station_IP; roslaunch rosbridge_server rosbridge_websocket.launch;"

# Start the user interface we use to control the robot
gnome-terminal -- bash -c "cd mars/mars-ui-web; source ../mars-ros/devel/setup.bash; npm start"

# Configure the existing terminal so that we can see information about ROS nodes and topics on the Jetson
cd mars/mars-ros
source devel/setup.zsh
export ROS_MASTER_URI=http://nano:11311
export ROS_IP=$control_station_IP
