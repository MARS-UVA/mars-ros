#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher scan_pub;

void depth_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg){
  ROS_INFO("inside callback");

  unsigned int num_readings = 100;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  //generate some fake data for our laser scan
  for(unsigned int i = 0; i < num_readings; ++i){
    ranges[i] = 1;
    intensities[i] = 100;
  }
  ros::Time scan_time = ros::Time::now();

  //populate the LaserScan message
  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  scan.header.frame_id = "dummy_laser_frame";
  scan.angle_min = -1.57;
  scan.angle_max = 1.57;
  scan.angle_increment = 3.14 / num_readings;
  scan.time_increment = (1 / laser_frequency) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 100.0;

  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for(unsigned int i = 0; i < num_readings; ++i){
    scan.ranges[i] = ranges[i];
    scan.intensities[i] = intensities[i];
  }

  scan_pub.publish(scan);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_laser_scan_publisher");

  ros::NodeHandle n;
  scan_pub = n.advertise<sensor_msgs::LaserScan>("dummy_scan", 50);
  ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 10, depth_cloud_callback);

  ros::spin();

  return 0;
}