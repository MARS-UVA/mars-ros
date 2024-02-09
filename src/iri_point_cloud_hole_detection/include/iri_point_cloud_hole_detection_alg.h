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

#ifndef _iri_point_cloud_hole_detection_alg_h_
#define _iri_point_cloud_hole_detection_alg_h_

#include <iri_point_cloud_hole_detection/PointCloudHoleDetectionConfig.h>
#include "mutex.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>

//include hole_detection_alg main library

/**
 * \brief IRI ROS Specific Driver Class
 *
 *
 */
class PointCloudHoleDetectionAlgorithm
{
  protected:
   /**
    * \brief define config type
    *
    * Define a Config type with the PointCloudHoleDetectionConfig. All driver implementations
    * will then use the same variable type Config.
    */
    CMutex alg_mutex_;

    // private attributes and methods
    pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for;   
  public:
   /**
    * \brief define config type
    *
    * Define a Config type with the PointCloudHoleDetectionConfig. All driver implementations
    * will then use the same variable type Config.
    */
    typedef iri_point_cloud_hole_detection::PointCloudHoleDetectionConfig Config;

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;

   /**
    * \brief constructor
    *
    * In this constructor parameters related to the specific driver can be
    * initalized. Those parameters can be also set in the openDriver() function.
    * Attributes from the main node driver class IriBaseDriver such as loop_rate,
    * may be also overload here.
    */
    PointCloudHoleDetectionAlgorithm(void);

   /**
    * \brief Lock Algorithm
    *
    * Locks access to the Algorithm class
    */
    void lock(void) { alg_mutex_.enter(); };

   /**
    * \brief Unlock Algorithm
    *
    * Unlocks access to the Algorithm class
    */
    void unlock(void) { alg_mutex_.exit(); };

   /**
    * \brief Tries Access to Algorithm
    *
    * Tries access to Algorithm
    * 
    * \return true if the lock was adquired, false otherwise
    */
    bool try_enter(void) { return alg_mutex_.try_enter(); };

   /**
    * \brief config update
    *
    * In this function the driver parameters must be updated with the input
    * config variable. Then the new configuration state will be stored in the 
    * Config attribute.
    *
    * \param new_cfg the new driver configuration state
    *
    * \param level level in which the update is taken place
    */
    void config_update(Config& new_cfg, uint32_t level=0);

    // here define all hole_detection_alg interface methods to retrieve and set
    // the driver parameters
    void cloud_all(pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out);
   /**
    * \brief Destructor
    *
    * This destructor is called when the object is about to be destroyed.
    *
    */
    ~PointCloudHoleDetectionAlgorithm(void);
};

#endif
