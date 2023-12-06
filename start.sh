<<instructions
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot).

Instructions: sudo ./start.sh or sudo ./start.sh <jetson's IP address>
instructions

# Define the IP address of the Jetson
jetsonIP=${1:-"192.168.0.2"}

# SSH into the Jetson and start the ROS nodes there
sshpass > /dev/null # try to run sshpass, but don't print out the text that you would normally see
if [ $? = 127 ] # check the last error code to see if it corresponds to the command not being installed
then
	echo "mars2324" | sudo -S apt install sshpass
fi
# gnome-terminal -- bash -c "sshpass -p nvidiauva ssh nvidia@192.168.0.2 \"docker run -d --network host malvi-config:1.0\""
# sshpass -p nvidiauva ssh nvidia@192.168.0.2 "docker run -d --network host malvi-config:1.0"

# Set environment variables that will allow the control station to find the Jetson on the network
cd
ethernetAddress=$(hostname -I)
nvidiaCount=`grep nvidia /etc/hosts -c`
if [ $nvidiaCount -gt 0 ]
then
	# sed -i "s/.* nvidia/$ethernetAddress nvidia/g" /etc/hosts
	echo "mars2324" | sudo -S sed -i "s/.* nvidia/$jetsonIP nvidia/g" /etc/hosts
else
	echo "$ethernetAddress nvidia" | cat >> /etc/hosts
fi
echo "Ethernet IP is $ethernetAddress"

# Start a rosbridge server to connect the control station to the ROS nodes on the Jetson
# gnome-terminal -- bash -c 'roslaunch rosbridge_server rosbridge_websocket.launch'
gnome-terminal -- bash -c "source mars/mars-ros/devel_isolated/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$jetsonIP; roslaunch rosbridge_server rosbridge_websocket.launch;"

# Start the user interface we use to control the robot
gnome-terminal -- bash -c 'cd mars/mars-ui-web; source ../mars-ros/devel_isolated/setup.bash; npm start'