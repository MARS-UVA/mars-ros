#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // used to convert tf2::Quaternion to geometry_msgs::Quaternion
#include <visualization_msgs/Marker.h>


// https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
visualization_msgs::Marker constructMarker(const XmlRpc::XmlRpcValue& value) {

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "tag_positions";
  marker.id = (int) value["id"];

  // Set the marker type.
  marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = (double) value["xpos"];
  marker.pose.position.y = (double) value["ypos"];
  marker.pose.position.z = (double) value["zpos"];

  tf2::Quaternion tq;
  tq.setRPY(0, 0, (double) value["yaw"]);
  geometry_msgs::Quaternion gq = tf2::toMsg(tq);
  marker.pose.orientation = gq;

  // Set the scale of the marker in meters
  marker.scale.x = 0.1;
  marker.scale.y = 0.02;
  marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0;
  marker.color.g = 0.7;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  return marker;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "environment_visualizer_node");

  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<visualization_msgs::Marker>("tag_positions", 10); // queue size

  while(publisher.getNumSubscribers() < 1) {
    if(!ros::ok()) {
      return 0;
    }
    sleep(1);
  }

  XmlRpc::XmlRpcValue standalone_positions;
  if(nodeHandle.getParam("tag_positions/standalone_tags", standalone_positions)) {
    ROS_INFO("Detected tag position subscriber, successfully loaded positions");
  } else {
    ROS_ERROR("Failed to load apriltag positions for visualization");
    return 1;
  }

  for(int i=0; i<standalone_positions.size(); i++) {
    publisher.publish(constructMarker(standalone_positions[i]));
  }

  ros::spin();
  return 0;
};