// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _iri_point_cloud_hole_detection_alg_node_h_
#define _iri_point_cloud_hole_detection_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "iri_point_cloud_hole_detection_alg.h"

#include <ros/ros.h>
//#include <ros/publisher.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>



// [publisher subscriber headers]
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class PointCloudHoleDetectionAlgNode : public algorithm_base::IriBaseAlgorithm<PointCloudHoleDetectionAlgorithm>
{
  private:
    // [publisher attributes]
    ros::Publisher hole_obstacle_publisher_;
    sensor_msgs::PointCloud2 PointCloud2_msg_;

    ros::Publisher hole_all_publisher_;    
    sensor_msgs::PointCloud2 PointCloud2_msg_all_;

    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in_rgb;
        
    // [subscriber attributes]
    ros::Subscriber input_subscriber_;
    void input_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    CMutex input_mutex_;

    // [service attributes]

    // [client attributes]

    // [action server attributes]
    
    // [action client attributes]

  public:
   /**
    * \brief Constructor
    * 
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    PointCloudHoleDetectionAlgNode(const ros::NodeHandle &nh=ros::NodeHandle("~"));

   /**
    * \brief Destructor
    * 
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~PointCloudHoleDetectionAlgNode(void);

  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency 
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects 
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    * 
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must 
    * implement it.
    *
    * \param config an object with new configuration from all algorithm 
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the 
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]
    
    // [test functions]
};

#include "nodelet/nodelet.h"
#include "threadserver.h"

class PointCloudHoleDetectionAlgNodelet : public nodelet::Nodelet
{
  private:
    PointCloudHoleDetectionAlgNode *node;
    virtual void onInit();// initialization function
    // thread attributes
    CThreadServer *thread_server;
    std::string spin_thread_id;
  protected:
    static void *spin_thread(void *param);
  public:
    PointCloudHoleDetectionAlgNodelet();

   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~PointCloudHoleDetectionAlgNodelet();
};


#endif
