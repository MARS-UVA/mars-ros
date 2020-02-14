#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_transforms");
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "camera_link";
    static_transformStamped.child_frame_id = "base_link";

    auto& trans = static_transformStamped.transform.translation;
    trans.x = 0;
    trans.y = 0;
    trans.z = 0;

    auto& rot = static_transformStamped.transform.rotation;
    rot.x = 0;
    rot.y = 0;
    rot.z = 0;
    rot.w = 1;
    static_broadcaster.sendTransform(static_transformStamped);

    ROS_INFO("Spinning until killed publishing to world");
    ros::spin();
    return 0;
};