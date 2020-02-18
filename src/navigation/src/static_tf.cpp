#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_transforms");
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // base to camera transform
    geometry_msgs::TransformStamped base_camera;

    base_camera.header.stamp = ros::Time::now();
    base_camera.header.frame_id = "base_link";
    base_camera.child_frame_id = "camera_link";

    auto& trans = base_camera.transform.translation;
    trans.x = 0;
    trans.y = 0;
    trans.z = 0;

    auto& rot = base_camera.transform.rotation;
    rot.x = 0;
    rot.y = 0;
    rot.z = 0;
    rot.w = 1;
    static_broadcaster.sendTransform(base_camera);

    ROS_INFO("Spinning until killed publishing to world");
    ros::spin();
    return 0;
};