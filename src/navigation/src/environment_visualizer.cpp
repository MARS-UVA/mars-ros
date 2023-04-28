#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // used to convert tf2::Quaternion to geometry_msgs::Quaternion
#include <visualization_msgs/Marker.h>


// https://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
visualization_msgs::Marker constructApriltagMarker(const XmlRpc::XmlRpcValue& value) {

  visualization_msgs::Marker apriltag_marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  apriltag_marker.header.frame_id = "map";
  apriltag_marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  apriltag_marker.ns = "tag_positions";
  apriltag_marker.id = (int) value["id"];

  // Set the marker type.
  apriltag_marker.type = visualization_msgs::Marker::CUBE;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  apriltag_marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  apriltag_marker.pose.position.x = (double) value["xpos"];
  apriltag_marker.pose.position.y = (double) value["ypos"];
  apriltag_marker.pose.position.z = (double) value["zpos"];

  tf2::Quaternion tq;
  tq.setRPY(0, 0, (double) value["yaw"]);
  geometry_msgs::Quaternion gq = tf2::toMsg(tq);
  apriltag_marker.pose.orientation = gq;

  /* April tag marker dimensions and color: */
  // Set the scale of the marker in meters
  apriltag_marker.scale.x = 0.1;
  apriltag_marker.scale.y = 0.02;
  apriltag_marker.scale.z = 0.1;

  // Set the color -- be sure to set alpha to something non-zero!
  apriltag_marker.color.r = 0.0;
  apriltag_marker.color.g = 0.7;
  apriltag_marker.color.b = 1.0;
  apriltag_marker.color.a = 1.0;

  apriltag_marker.lifetime = ros::Duration();

  return apriltag_marker;
}

visualization_msgs::Marker constructBinMarker(const XmlRpc::XmlRpcValue& value) {

  visualization_msgs::Marker bin_marker;
  bin_marker.header.frame_id = "map";
  bin_marker.header.stamp = ros::Time::now();

  bin_marker.ns = "bin_position";
  bin_marker.id = 0;

  bin_marker.type = visualization_msgs::Marker::CUBE;
  bin_marker.action = visualization_msgs::Marker::ADD;

  bin_marker.pose.position.x = (double) value["xpos"];
  bin_marker.pose.position.y = (double) value["ypos"];
  bin_marker.pose.position.z = (double) value["zpos"];

  tf2::Quaternion tq;
  tq.setRPY(0, 0, 0);
  geometry_msgs::Quaternion gq = tf2::toMsg(tq);
  bin_marker.pose.orientation = gq;

  bin_marker.scale.x = 0.1;
  bin_marker.scale.y = 0.5;
  bin_marker.scale.z = 0.1;

  bin_marker.color.r = 1.0;
  bin_marker.color.g = 0.2;
  bin_marker.color.b = 0.3;
  bin_marker.color.a = 1.0;

  bin_marker.lifetime = ros::Duration();

  return bin_marker;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "environment_visualizer_node");

  ros::NodeHandle nodeHandle;
  ros::Publisher publisher = nodeHandle.advertise<visualization_msgs::Marker>("positions", 1); // queue size

  XmlRpc::XmlRpcValue standalone_apriltag_positions;
  XmlRpc::XmlRpcValue bin_position;
  if(nodeHandle.getParam("tag_positions/standalone_tags", standalone_apriltag_positions)) {
    ROS_INFO("Detected tag position subscriber, successfully loaded apriltag positions");
  } else {
    ROS_ERROR("Failed to load apriltag positions for visualization");
    return 1;
  }
 
  if(nodeHandle.getParam("bin_position", bin_position)) {
    ROS_INFO("Detected bin position subscriber, successfully loaded bin position");
  } else {
    ROS_ERROR("Failed to load bin positions for visualization");
    return 1;
  }


  // wait until there's a subscriber to the topic before publishing
  while(publisher.getNumSubscribers() < 1) {
    if(!nodeHandle.ok()) {
      return 0;
    }
    sleep(1);
  }

  // once there's a subscriber, publish periodically just for good measure (ie if someone new subscribes) even though the data isn't changing
  ros::Rate r(0.2); // 0.2 hz = every 5 seconds
  while (nodeHandle.ok()) {
    for(int i=0; i<standalone_apriltag_positions.size(); i++) {
      publisher.publish(constructApriltagMarker(standalone_apriltag_positions[i]));
    }
    
    publisher.publish(constructBinMarker(bin_position));

    ros::spinOnce();
    r.sleep();
  }

  return 0;
};
