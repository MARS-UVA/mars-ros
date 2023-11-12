#Instructions: sudo ./start.sh <jetson's IP address>
#Step 5 - Probably done, not sure how to test
jetsonIP=$1
sshpass
if [ $? = 127 ]
then
	apt install sshpass
fi

sshpass -p nvidiauva ssh nvidia@$jetsonIP 'cd mars-ros; ./setup.sh; source devel_isolated/setup.bash; roslaunch navigation malvi_config.launch'
echo "Done with SSH"

#Step 6 - Tested
cd
ethernetAddress=`ifconfig | grep -o ....::....:....:....:....`
nvidiaCount=`grep nvidia /etc/hosts -c`
if [ $nvidiaCount -gt 0 ]
then
	sed -i "s/.* nvidia/$ethernetAddress nvidia/g" /etc/hosts
else
	echo "$ethernetAddress nvidia" | cat >> /etc/hosts
fi
echo "Ethernet IP is $ethernetAddress"


#Step 7 - currently broken
gnome-terminal -e "bash -c 'source ../mars-ros/devel_isolated/actions/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$jetsonIP; roslaunch rosbridge_server rosbridge_websocket.launch;sleep 20'"


#Step 8 - Tested
gnome-terminal -e "bash -c 'pwd; cd mars/mars-ros; cd ../mars-ui-web; source ../mars-ros/devel/setup.bash; npm start; sleep 20'"

