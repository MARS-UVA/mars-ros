// #include <apriltag_ros/AprilTagDetectionArray.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // used to convert tf2::Quaternion to geometry_msgs::Quaternion
#include <geometry_msgs/Twist.h>
#include <cmath>


class Localization {
public:
  geometry_msgs::TransformStamped currentPosition;
  nav_msgs::Odometry currentOdom;

  Localization(ros::NodeHandle&);
  void updateCurrentPosition();
  void cmdVelCallback(const geometry_msgs::Twist&);
//   void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray&);

private:
  geometry_msgs::Twist lastCommand; // units are m/s and rad/s
  float currentYaw; // RPY needs to be separated because TransformStamped only stores quaternion rotation
  ros::Time lastUpdateTime;

  ros::NodeHandle& nodeHandle;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

//   geometry_msgs::PointStamped getTagConfigPosition(int);
};


Localization::Localization(ros::NodeHandle& nh) :
  nodeHandle(nh), 
  tfListener(tfBuffer) {
  
  lastCommand.linear.x = 0;
  lastCommand.angular.z = 0;

  currentPosition.header.frame_id = "odom";
  currentPosition.child_frame_id = "base_link";
  currentPosition.transform.translation.x = 0.0;
  currentPosition.transform.translation.y = 0.0;
  currentPosition.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  currentPosition.transform.rotation.x = q.x();
  currentPosition.transform.rotation.y = q.y();
  currentPosition.transform.rotation.z = q.z();
  currentPosition.transform.rotation.w = q.w();
  currentYaw = 0;

  currentOdom.header.frame_id = "odom";
  currentOdom.child_frame_id = "base_link";
  currentOdom.pose.pose.position.x = 0.0;
  currentOdom.pose.pose.position.y = 0.0;
  currentOdom.pose.pose.position.z = 0.0;
  currentOdom.pose.pose.orientation = tf2::toMsg(q);
  currentOdom.twist.twist.linear.x = 0.0;
  currentOdom.twist.twist.linear.y = 0.0;
  currentOdom.twist.twist.linear.z = 0.0;
  currentOdom.twist.twist.angular.z = 0.0;
  currentOdom.twist.twist.angular.y = 0.0;
  currentOdom.twist.twist.angular.z = 0.0;

//   if(nodeHandle.getParam("tag_positions/standalone_tags", standalone_positions)) {
//     // ROS_INFO("Detected tag position subscriber, successfully loaded positions");
//   } else {
//     ROS_ERROR("Failed to load apriltag positions from config for localization");
//   }
}

// geometry_msgs::PointStamped Localization::getTagConfigPosition(int id) {
//   geometry_msgs::PointStamped r;

//   for(int i=0; i<standalone_positions.size(); i++) {
//     if(id == (int) standalone_positions[i]["id"]) {
//       r.header.frame_id = "map";
//       r.point.x = (double) standalone_positions[i]["xpos"];
//       r.point.y = (double) standalone_positions[i]["ypos"];
//       r.point.z = (double) standalone_positions[i]["zpos"];
//       // TODO set timestamp?
//       r.header.stamp = ros::Time::now();
//       return r;
//     }
//   }

//   ROS_ERROR("Failed to find apriltag position with id=%d to perform localization", id);
//   return r;
// }

void Localization::updateCurrentPosition() {
  ros::Time currentTime = ros::Time::now();
  double dt = currentTime.toSec() - lastUpdateTime.toSec();

  currentPosition.header.stamp = currentTime;
  currentYaw += lastCommand.angular.z * dt;
  tf2::Quaternion q;
  q.setRPY(0, 0, currentYaw);
  currentPosition.transform.rotation.x = q.x();
  currentPosition.transform.rotation.y = q.y();
  currentPosition.transform.rotation.z = q.z();
  currentPosition.transform.rotation.w = q.w();
  currentPosition.transform.translation.x += cos(currentYaw) * lastCommand.linear.x * dt;
  currentPosition.transform.translation.y += sin(currentYaw) * lastCommand.linear.x * dt;

  currentOdom.header.stamp = currentTime;
  currentOdom.pose.pose.position.x = currentPosition.transform.translation.x;
  currentOdom.pose.pose.position.y = currentPosition.transform.translation.y;
  currentOdom.pose.pose.orientation = tf2::toMsg(q);
  currentOdom.twist.twist.linear.x = lastCommand.linear.x;
  currentOdom.twist.twist.angular.z = lastCommand.angular.z;

  lastUpdateTime = currentTime;
}

void Localization::cmdVelCallback(const geometry_msgs::Twist& msg){
  // ROS_INFO("Updating velocity: x=%f, y=%f, yaw=%f", msg.linear.x, msg.linear.y, msg.angular.z);
  updateCurrentPosition();
  lastCommand = msg;
}

// void Localization::tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray& msg) {
//   int count = msg.detections.size();
//   if(count == 0) {
//     return;
//   }

//   float avgX = 0;
//   float avgY = 0;
//   std::string frame_id = msg.header.frame_id;

//   for(int i=0; i<count; i++) {

//     // AprilTagDetectionArray -> AprilTagDetection -> PoseWithCovarianceStamped -> PoseWithCovariance -> Pose
//     int id = msg.detections[i].id[0];
//     geometry_msgs::PointStamped tagDetectedCamera;
//     // tagDetectedCamera.header.frame_id = frame_id;
//     tagDetectedCamera.header.frame_id = "camera";
//     tagDetectedCamera.header.stamp = msg.header.stamp;
//     tagDetectedCamera.point = msg.detections[i].pose.pose.pose.position;
//     // ROS_INFO("Got detections (#%d): f=%s, id=%d, p=(%f, %f, %f)", i, frame_id, id, tagDetectedCamera.point.x, tagDetectedCamera.point.y, tagDetectedCamera.point.z);
//     ROS_INFO("Got detections, tagDetectedCamera=(%f, %f, %f)", tagDetectedCamera.point.x, tagDetectedCamera.point.y, tagDetectedCamera.point.z);

//     // Convert detected tag position to odom frame
//     geometry_msgs::PointStamped tagDetectedOdom;
//     // tfBuffer.transform(tagDetectedCamera, tagDetectedOdom, "odom");
//     // Another method of transforming: 
//     geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(frame_id, "odom", msg.header.stamp, ros::Duration(0.1));
//     tf2::doTransform(tagDetectedCamera, tagDetectedOdom, transform);
//     ROS_INFO("  tagDetectedOdom=(%f, %f, %f)", tagDetectedOdom.point.x, tagDetectedOdom.point.y, tagDetectedOdom.point.z);

//     // Convert config tag position to odom frame
//     geometry_msgs::PointStamped tagConfigMap = getTagConfigPosition(id);
//     ROS_INFO("  tagConfigMap=(%f, %f, %f)", tagConfigMap.point.x, tagConfigMap.point.y, tagConfigMap.point.z);
//     geometry_msgs::PointStamped tagConfigOdom;
//     tfBuffer.transform(tagConfigMap, tagConfigOdom, "odom");
//     ROS_INFO("  tagConfigOdom=(%f, %f, %f)", tagConfigOdom.point.x, tagConfigOdom.point.y, tagConfigOdom.point.z);

//     // Perform vector arithmetic: robot = detected - actual
//     avgX += tagDetectedOdom.point.x - tagConfigOdom.point.x;
//     avgY += tagDetectedOdom.point.y - tagConfigOdom.point.y;
//   }

//   avgX /= count;
//   avgY /= count;

//   currentPosition.transform.translation.x = avgX;
//   currentPosition.transform.translation.y = avgY;
// }

int main(int argc, char** argv){
  ros::init(argc, argv, "localization_node");
  
  ros::NodeHandle nodeHandle;
  Localization localization(nodeHandle);

  // See https://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
  ros::Subscriber cmdVelSub = nodeHandle.subscribe("/cmd_vel", 0, &Localization::cmdVelCallback, &localization); // queue size
//   ros::Subscriber tagDetectionsSub = nodeHandle.subscribe("/tag_detections", 0, &Localization::tagDetectionsCallback, &localization);
  tf2_ros::TransformBroadcaster transformBroadcaster;
  ros::Publisher odomPublisher = nodeHandle.advertise<nav_msgs::Odometry>("odom", 10);

  ROS_INFO("Subscribed to /cmd_vel, starting to broadcast+publish in loop");

  ros::Rate loop_rate(15);
  while (ros::ok()) {
    localization.updateCurrentPosition();
    transformBroadcaster.sendTransform(localization.currentPosition);
    odomPublisher.publish(localization.currentOdom);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
};