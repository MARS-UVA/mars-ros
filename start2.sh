<<instructions
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot). This should be run AFTER start1.sh.

Instructions: ./start2.sh or ./start2.sh <jetson's IP address>
instructions

# Define the IP address of the Jetson
JETSON_IP=${1:-"192.168.0.2"}

# Set environment variables that will allow the control station to find the Jetson on the network
cd
control_station_IP=$(hostname -I)
nvidia_count=`grep nvidia /etc/hosts -c`

if [ $nvidia_count -gt 0 ]
then
	echo "mars2324" | sudo -S sed -i "s/.* nvidia/$JETSON_IP nvidia/g" /etc/hosts
else
	echo "mars2324" | sudo -S sed -i "/The following*/i $control_station_IP nvidia\n" /etc/hosts
fi

# Start a rosbridge server to connect the control station to the ROS nodes on the Jetson
gnome-terminal -- bash -c "source mars/mars-ros/devel_isolated/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$control_station_IP; roslaunch rosbridge_server rosbridge_websocket.launch;"

# Start the user interface we use to control the robot
gnome-terminal -- bash -c "cd mars/mars-ui-web; source ../mars-ros/devel_isolated/setup.bash; npm start"