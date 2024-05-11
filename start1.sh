#!/bin/zsh

<<instructions
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot). This should be run BEFORE start2.sh.

Instructions: source start1.sh 
					or 
			  source start1.sh <jetson's IP address>
instructions

# Define the IP address of the Jetson
JETSON_IP=${1:-"192.168.0.102"}

# SSH into the Jetson and start the ROS nodes there
sshpass > /dev/null # try to run sshpass, but don't print out the text that you would normally see
if [ $? = 127 ] # check the last error code to see if it corresponds to the command not being installedgit fetch
then
	echo "mars2324" | sudo -S apt install sshpass
fi

# sshpass -p nvidiauva ssh -tt nvidia@$JETSON_IP "docker run --rm -it --network host --volume=/dev:/dev --privileged main:3"
sshpass -p <password> ssh -tt jetson@$JETSON_IP "cd mars/mars-ros; git fetch; source devel/setup.bash; roslaunch navigation malvi_config.launch apriltag_camera_device_id:=0"
