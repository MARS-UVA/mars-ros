PREFIX=${1:-"/home/madelyn/mars"}
export GAZEBO_MODEL_PATH="$PREFIX/gazebo_apriltag/models"
roslaunch navigation apriltag_simulation.launch