<<instructions
Run this script on the control station when you want to connect the control station 
to the Jetson (the computer onboard the robot). This should be run BEFORE start2.sh.

Instructions: ./start2.sh or ./start2.sh <jetson's IP address>
instructions

# Define the IP address of the Jetson
JETSON_IP=${1:-"192.168.0.2"}

# SSH into the Jetson and start the ROS nodes there
sshpass > /dev/null # try to run sshpass, but don't print out the text that you would normally see
if [ $? = 127 ] # check the last error code to see if it corresponds to the command not being installed
then
	echo "mars2324" | sudo -S apt install sshpass
fi

sshpass -p nvidiauva ssh -tt nvidia@$JETSON_IP "docker run -it --network host malvi-config:1.0"