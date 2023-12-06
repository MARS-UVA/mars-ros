# SSH into the Jetson and start the ROS nodes there
if [ $? = 127 ]
then
	apt install sshpass
fi


# gnome-terminal -- bash -c "sshpass -p nvidiauva ssh nvidia@192.168.0.2 \"docker run -d --network host malvi-config:1.0\""