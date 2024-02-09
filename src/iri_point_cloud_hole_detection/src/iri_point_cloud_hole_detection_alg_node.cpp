#include "iri_point_cloud_hole_detection_alg_node.h"

PointCloudHoleDetectionAlgNode::PointCloudHoleDetectionAlgNode(const ros::NodeHandle &nh) :
  algorithm_base::IriBaseAlgorithm<PointCloudHoleDetectionAlgorithm>(nh)
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->hole_obstacle_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("hole_obstacle", 1);
  this->hole_all_publisher_ = this->public_node_handle_.advertise<sensor_msgs::PointCloud2>("hole_zone", 1);
  // [init subscribers]
  this->input_subscriber_ = this->public_node_handle_.subscribe("input", 1, &PointCloudHoleDetectionAlgNode::input_callback, this);


  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

PointCloudHoleDetectionAlgNode::~PointCloudHoleDetectionAlgNode(void)
{
  // [free dynamic memory]
}

void PointCloudHoleDetectionAlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  //this->PointCloud2_msg_.data = my_var;

  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */
void PointCloudHoleDetectionAlgNode::input_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_DEBUG("PointCloudHoleDetectionAlgNode::input_callback: New Message Received");

  pcl::fromROSMsg(*msg,this->cloud_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for(new pcl::PointCloud<pcl::PointXYZ>);

  this->alg_.lock();

  this->cloud_in_rgb.points.resize(this->cloud_in.size());
  this->cloud_in_rgb.width=this->cloud_in.width;
  this->cloud_in_rgb.height=this->cloud_in.height;
  for(size_t i=0;i<this->cloud_in.points.size();i++)
  {
    this->cloud_in_rgb.points[i].x=this->cloud_in.points[i].x;
    this->cloud_in_rgb.points[i].y=this->cloud_in.points[i].y;
    this->cloud_in_rgb.points[i].z=this->cloud_in.points[i].z;
    this->cloud_in_rgb.points[i].r=0;
    this->cloud_in_rgb.points[i].g=0;
    this->cloud_in_rgb.points[i].b=0;
  }

  this->alg_.cloud_all(cloud_in_rgb, cloud_for);
  //unlock previously blocked shared variables
  this->alg_.unlock();

///////////////////////////////////////////////////////////////////////////////////////

  pcl::toROSMsg(*cloud_for,this->PointCloud2_msg_);
  this->PointCloud2_msg_.header=msg->header;
  pcl::toROSMsg(this->cloud_in_rgb,this->PointCloud2_msg_all_);
  this->PointCloud2_msg_all_.header=msg->header;

  this->hole_obstacle_publisher_.publish(this->PointCloud2_msg_);
  this->hole_all_publisher_.publish(this->PointCloud2_msg_all_);
}

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void PointCloudHoleDetectionAlgNode::node_config_update(Config &config, uint32_t level)
{
}

void PointCloudHoleDetectionAlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<PointCloudHoleDetectionAlgNode>(argc, argv, "iri_point_cloud_hole_detection_alg_node");
}

#include <pluginlib/class_list_macros.h>

PointCloudHoleDetectionAlgNodelet::PointCloudHoleDetectionAlgNodelet()
{
  this->node=NULL;
}

PointCloudHoleDetectionAlgNodelet::~PointCloudHoleDetectionAlgNodelet(void)
{
  // kill the thread
  this->thread_server->kill_thread(this->spin_thread_id);
  this->thread_server->detach_thread(this->spin_thread_id);
  this->thread_server->delete_thread(this->spin_thread_id);
  this->spin_thread_id="";
  // [free dynamic memory]
  if(this->node!=NULL)
    delete this->node;
}

void PointCloudHoleDetectionAlgNodelet::onInit()
{
  // initialize the driver node
  this->node=new PointCloudHoleDetectionAlgNode(getPrivateNodeHandle());
  // initialize the thread
  this->thread_server=CThreadServer::instance();
  this->spin_thread_id=getName() + "_point_cloud_hole_detector_spin";
  this->thread_server->create_thread(this->spin_thread_id);
  this->thread_server->attach_thread(this->spin_thread_id,this->spin_thread,this);
  // start the spin thread
  this->thread_server->start_thread(this->spin_thread_id);
}

void *PointCloudHoleDetectionAlgNodelet::spin_thread(void *param)
{
  PointCloudHoleDetectionAlgNodelet *nodelet=(PointCloudHoleDetectionAlgNodelet *)param;

  nodelet->node->nodelet_spin();

  pthread_exit(NULL);
}

// parameters are: package, class name, class type, base class type
PLUGINLIB_EXPORT_CLASS(PointCloudHoleDetectionAlgNodelet, nodelet::Nodelet);

