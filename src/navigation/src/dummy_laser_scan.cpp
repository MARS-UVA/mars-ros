#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
// #include <sensor_msgs/point_cloud2.h>

ros::Publisher scan_pub;

void depth_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  ROS_INFO("inside callback");

  //populate the dummy LaserScan message

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

  // handle the point cloud

  sensor_msgs::PointCloud2 msg = *depth_cloud_msg;
  
  // height: 1080, width: 1920
  // row step: 61440 ( = width * point step = 1920 * 32)
  // the fields are x, y, z, and rgb, each of which seems to be 4 bytes long except for rgb which is probably 16
  // point step is 32 bytes total

  // iterate over the point cloud --> each iterator iterates over the specified field of each point in the cloud

  sensor_msgs::PointCloud2ConstIterator<float> xIt(*depth_cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> yIt(*depth_cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> zIt(*depth_cloud_msg, "z");
  for (xIt; xIt != xIt.end(); ++xIt) {
    ++yIt;
    ++zIt;
  }

  scan_pub.publish(scan);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "dummy_laser_scan_publisher");

  ros::NodeHandle n;
  scan_pub = n.advertise<sensor_msgs::LaserScan>("dummy_scan", 50);
  ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 1, depth_cloud_callback);

  ros::spin();

  return 0;
}