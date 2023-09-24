#Instructions: sudo ./start.sh <jetson's IP address>
#Step 5 - authorization probably doesn't work
gnome-terminal -e "bash -c 'echo "nvidiauva" | ssh nvidia@$1; cd mars-ros; ./setup.sh; roslaunch navigation malvi_config.launch;'"


#Step 6 - Tested
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


#Step 7
gnome-terminal -e "bash -c 'source ../mars-ros/devel_isolated/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$ethernetAddress; roslaunch rosbridge_server rosbridge_websocket.launch;'"


#Step 8
gnome-terminal -e "bash -c 'cd mars/mars-ui-web; source ../mars-ros/devel_isolated/setup.bash; npm start'"
