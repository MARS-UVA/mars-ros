#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <math.h>

ros::Publisher scan_pub;
double cam_fov = -1.0; // a positive angle (in radians) indicating the field of view of the camera
double camera_noise_in_meters = 0.00;
/*
Converts a PointCloud2 message into a LaserScan message.

The point cloud for the simulated depth camera has:
height: 1080
width: 1920
row step: 61440 (width * point step = 1920 * 32)
fields: x (4 bytes), y (4 bytes), z (4 bytes), rgb
point step: 32 bytes
*/
void depth_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& depth_cloud_msg) {
  
  if(cam_fov < 0) {
    ROS_INFO("Camera FOV not yet set.");
    return;
  }

  unsigned int num_readings = 180;
  double a_ranges[num_readings]; // strategy A, farthest point
  double b_ranges[num_readings]; // strategy B, virtual floor projection
  double intensities[num_readings];

  ros::Time scan_time = ros::Time::now();

  sensor_msgs::LaserScan scan;
  scan.header.stamp = scan_time;
  scan.header.frame_id = "base_scan";
  scan.angle_min = -1 * (cam_fov/2);
  scan.angle_max = (cam_fov/2);
  scan.angle_increment = (scan.angle_max - scan.angle_min) / num_readings;
  scan.time_increment = (1/300); // set as 1/60 because simulated depth camera (rs200) can achieve 60 fps // (1 / laser_frequency) / (num_readings);
  scan.range_min = 0.0;
  scan.range_max = 10.0;

  for(unsigned int i = 0; i < num_readings; i++) {
    a_ranges[i] = scan.range_min;
    b_ranges[i] = scan.range_max;
    intensities[i] = scan.range_max;
  }

  sensor_msgs::PointCloud2 msg = *depth_cloud_msg;

  // iterate over the point cloud --> each iterator iterates over the specified field of each point in the cloud
  double angle;
  int step;
  double a_range, b_range;
  // double x, y, z;
  double xPrime, yPrime, zPrime, y;

  yPrime = 0.094; // how high the depth camera is above the ground, based on the turtlebot waffle urdf file

  sensor_msgs::PointCloud2ConstIterator<float> xIt(*depth_cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> yIt(*depth_cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> zIt(*depth_cloud_msg, "z");
  for (xIt; xIt != xIt.end(); ++xIt) {
    if(!isnan(xIt[0])) { // only look at points with valid readings
      // ROS_INFO_STREAM("point: ("<<xIt[0] << ", " << yIt[0] << ", " << zIt[0]);
      
      // x goes forward, z to right, y upward
      angle = atan(xIt[0]/(-zIt[0])); // angle = arctan(x/z)
      step = (angle - scan.angle_min)/ scan.angle_increment;

      // strategy a
      a_range = sqrt(pow(zIt[0], 2) + pow(xIt[0], 2));
      
      if(a_ranges[step] < a_range) {
        a_ranges[step] = a_range;
      }

      // strategy b - only look at points below the floor plane
      // currently not working
      if(yIt[0] >  camera_noise_in_meters) {
	y = yIt[0] + yPrime;
        xPrime = yPrime * xIt[0] / y; // x' = y' * x/y
        //zPrime = xPrime * (-zIt[0]) / xIt[0]; // z' = x' * z/x

        b_range = sqrt(pow(zIt[0], 2) + pow(xPrime, 2)); //new pts

        if(b_ranges[step] > b_range) {
          b_ranges[step] = b_range;
        }
      }
      
    }

    ++yIt;
    ++zIt;
  }

  scan.ranges.resize(num_readings);
  scan.intensities.resize(num_readings);
  for(unsigned int i = 0; i < num_readings; ++i){
    scan.ranges[i] = b_ranges[i];
    scan.intensities[i] = intensities[i];
    // ROS_INFO_STREAM(i<<": "<< scan.ranges[i]);
  }

  scan_pub.publish(scan);
}

double calculateAngle(sensor_msgs::CameraInfoConstPtr cam_info) {
  // calculation function from https://stackoverflow.com/questions/39992968/how-to-calculate-field-of-view-of-the-camera-from-camera-intrinsic-matrix
  // info about theoretical camera FOV on page 15 of https://www.mouser.com/pdfdocs/intel_realsense_camera_r200.pdf 

  sensor_msgs::CameraInfo info = *cam_info;
  ROS_INFO_STREAM("calculating camera FOV from the following camera calibration info:\n"<< info);

  double f_x = info.K[0]; // focal length of camera along x dimension

  double fov_x = 2 * atan(info.width / (2 * f_x)); // field of view of camera along x axis
  return fov_x;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "dummy_laser_scan_publisher");

  ros::NodeHandle n;

  // note: camera info topic is only published while /camera/depth/points is being subscribed to

  scan_pub = n.advertise<sensor_msgs::LaserScan>("dummy_scan", 1);
  ros::Subscriber point_cloud_sub = n.subscribe("/camera/depth/points", 1, depth_cloud_callback);

  // get the depth camera info (only once upon startup) and use it to calculate the field of view that the camera can sense;
  // this will be relevant for knowing which angles to publish the LaserScan at
  sensor_msgs::CameraInfoConstPtr cam_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info");
  cam_fov = calculateAngle(cam_info);

  ros::spin();

  return 0;
}
