#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void setTranslation(geometry_msgs::Vector3& trans, double x = 0.0, double y = 0.0, double z = 0.0) {
    trans.x = x;
    trans.y = y;
    trans.z = z;
}

void setRotation(geometry_msgs::Quaternion& quat, double r = 0.0, double p = 0.0, double y = 0.0) {
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(r, p, y);
    tf2::convert(quat_tf, quat);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_transforms");
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // base to camera transform
    geometry_msgs::TransformStamped base_camera;
    // map to odom transform
    geometry_msgs::TransformStamped map_odom;

    base_camera.header.stamp = ros::Time::now();
    base_camera.header.frame_id = "base_link";
    base_camera.child_frame_id = "camera_link";

    map_odom.header.stamp = ros::Time::now();
    map_odom.header.frame_id = "map";
    map_odom.child_frame_id = "odom";

    setTranslation(base_camera.transform.translation);
    setRotation(base_camera.transform.rotation);
    static_broadcaster.sendTransform(base_camera);

    setTranslation(map_odom.transform.translation);
    setRotation(map_odom.transform.rotation);
    static_broadcaster.sendTransform(map_odom);

    ROS_INFO("Spinning until killed publishing to world");
    ros::spin();
    return 0;
};