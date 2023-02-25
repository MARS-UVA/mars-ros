#include <ros/ros.h>
#include <image_transport/image_transport.h>
// #include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// https://docs.ros.org/en/api/sensor_msgs/html/msg/CompressedImage.html

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
    cv::Mat img_resized;
    cv::resize(img, img_resized, cv::Size(150, 150));

    // cv::Mat img_compressed = 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_streamer_node");
    ros::NodeHandle nh;
    // cv::namedWindow("view");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    // cv::destroyWindow("view");
}