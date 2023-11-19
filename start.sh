#Instructions: ./start.sh
#Step 5 - Tested
jetsonIP="192.168.0.102"
sshpass
if [ $? = 127 ]
then
	echo "mars2324" | sudo -S apt install sshpass
fi

#sshpass -p nvidiauva ssh nvidia@$jetsonIP 'cd mars-ros; source devel_isolated/setup.bash; roslaunch navigation malvi_config.launch'
echo "Done with SSH"

#Step 6 - Tested
cd
ethernetAddress=`hostname -I`
nvidiaCount=`grep nvidia /etc/hosts -c`
if [ $nvidiaCount -gt 0 ]
then
	echo "mars2324" | sudo -S sed -i "s/.* nvidia/$jetsonIP nvidia/g" /etc/hosts
else
	echo "mars2324" | sudo -S -- sh -c -e "echo '$jetsonIP nvidia' >> /etc/hosts"
fi
echo "Ethernet IP is $ethernetAddress"


#Step 7 - Tested
gnome-terminal -e "bash -c 'pwd; cd mars/mars-ui-web; source ../mars-ros/devel/setup.bash; export ROS_MASTER_URI=http://nvidia:11311; export ROS_IP=$ethernetAddress; roslaunch rosbridge_server rosbridge_websocket.launch;sleep 20'"


#Step 8 - Tested
gnome-terminal -e "bash -c 'pwd; cd mars/mars-ros; cd ../mars-ui-web; source ../mars-ros/devel/setup.bash; npm start; sleep 20'"
