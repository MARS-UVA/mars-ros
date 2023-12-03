"""
#Step 5 - Tested	Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot).

Instructions: sudo ./start.sh or sudo ./start.sh <jetson's IP address>
"""

# Define the IP address of the Jetson
jetsonIP=${1:-"192.168.0.2"}

# SSH into the Jetson and start the ROS nodes there
sshpass
if [ $? = 127 ]
then
	apt install sshpass
fi
gnome-terminal -e "bash -c \"sshpass -p nvidiauva ssh nvidia@$jetsonIP \"docker run -it --network host malvi-config:1.0\"\""

# Set environment variables that will allow the control station to find the Jetson on the network
cd
ethernetAddress=`ifconfig | grep -o ..:..:..:..:..:..`
nvidiaCount=`grep nvidia /etc/hosts -c`
if [ $nvidiaCount -gt 0 ]
then
	sed -i "s/.* nvidia/$ethernetAddress nvidia/g" /etc/hosts
else
	echo "$ethernetAddress nvidia" | cat >> /etc/hosts
fi
echo "Ethernet IP is $ethernetAddress"

# Start a rosbridge server to connect the control station to the ROS nodes on the Jetson
gnome-terminal -e "bash -c 'source ../mars-ros/devel_isolated/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$jetsonIP; roslaunch rosbridge_server rosbridge_websocket.launch;'"

# Start the user interface we use to control the robot
gnome-terminal -e "bash -c 'cd mars/mars-ui-web; source ../mars-ros/devel_isolated/setup.bash; npm start'"