"""
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot).
"""

# Define the IP address of the Jetson
jetsonIP="192.168.0.102"

# SSH into the Jetson
sshpass
if [ $? = 127 ] # checks if sshpass is installed
then
	echo "mars2324" | sudo -S apt install sshpass
fi
#sshpass -p nvidiauva ssh nvidia@$jetsonIP 'cd mars-ros; source devel_isolated/setup.bash; roslaunch navigation malvi_config.launch' # starts our main launch file on the robot
echo "Done with SSH"

# set environment variables that will allow the control station to find the Jetson on the network
cd
ethernetAddress=`hostname -I`
nvidiaCount=`grep nvidia /etc/hosts -c` # counts the number of times that "nvidia" is defined in the /etc/hosts file
if [ $nvidiaCount -gt 0 ]
then
	echo "mars2324" | sudo -S sed -i "s/.* nvidia/$jetsonIP nvidia/g" /etc/hosts
else
	echo "mars2324" | sudo -S -- sh -c -e "echo '$jetsonIP nvidia' >> /etc/hosts"
fi
echo "Ethernet IP is $ethernetAddress"


# Start a rosbridge server to connect the control station to the ROS nodes on the Jetson
gnome-terminal -e "bash -c 'pwd; cd mars/mars-ui-web; source ../mars-ros/devel/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$ethernetAddress; roslaunch rosbridge_server rosbridge_websocket.launch;sleep 20'"


# Start the user interface we use to control the robot
gnome-terminal -e "bash -c 'pwd; cd mars/mars-ros; cd ../mars-ui-web; source ../mars-ros/devel/setup.bash; npm start; sleep 20'"
