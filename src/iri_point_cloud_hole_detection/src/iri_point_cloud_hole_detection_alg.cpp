#include "iri_point_cloud_hole_detection_alg.h"
using namespace std;

PointCloudHoleDetectionAlgorithm::PointCloudHoleDetectionAlgorithm(void)
{
}

PointCloudHoleDetectionAlgorithm::~PointCloudHoleDetectionAlgorithm(void)
{
}

void PointCloudHoleDetectionAlgorithm::config_update(Config& new_cfg, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_=new_cfg;

  this->unlock();
}

// PointCloudHoleDetectionAlgorithm Public API
/*
It selects the points inside a box generated using the input params. These points
are separated in three regions. If any of them has less than a minimum number of
points, there is a hole so it generates a virtual obstacle in order to notify
the navigation stack.
*/
void PointCloudHoleDetectionAlgorithm::cloud_all(pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_for)
{
  int p [this->config_.num_cells]={0};
  float threshold[this->config_.num_cells+1]={0};
  float Z;

  //////////////////////////////////////////////////////////////////////////////
  // Hole detection box generation
  //////////////////////////////////////////////////////////////////////////////
  for(int i=0;i<=this->config_.num_cells;++i)
    threshold[i]=(this->config_.box_y/2)-((this->config_.num_cells-i)*(this->config_.box_y/this->config_.num_cells));

  for(size_t rowIndex=0,pointIndex=0;rowIndex<cloud_in.height;++rowIndex)
  {
    for(size_t colIndex=0;colIndex<cloud_in.width;++colIndex,++pointIndex)
    {
//      if(this->config_.box_z_ini<cloud_in.points[pointIndex].z &&
//         cloud_in.points[pointIndex].z<this->config_.box_z_end &&
//         this->config_.box_x_ini<cloud_in.points[pointIndex].x &&
//         cloud_in.points[pointIndex].x<this->config_.box_x_end)
      if(this->config_.box_x_ini<cloud_in.points[pointIndex].x &&
         cloud_in.points[pointIndex].x<this->config_.box_x_end)
      {
        for(int cell=0;cell<this->config_.num_cells;++cell)
        {
          if(threshold[cell]<cloud_in.points[pointIndex].y &&
             cloud_in.points[pointIndex].y<threshold[cell+1])
          {
            cloud_in.points[pointIndex].r=0;
            cloud_in.points[pointIndex].g=int(255*(float(cell)/float(max(1,this->config_.num_cells-1))));
            cloud_in.points[pointIndex].b=255;
            p[cell]=p[cell]+1;
          }
        }
      } //if() bracket
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Virtual obstacle generation
  //////////////////////////////////////////////////////////////////////////////
  Z=0.1;

  for(size_t rowIndex=0,pointIndex=0;rowIndex<5;++rowIndex)
  {
    for(size_t colIndex=0;colIndex<5;++colIndex,++pointIndex)
    {
      for(int cell=0;cell<this->config_.num_cells; ++cell)
      {
        if(p[cell]<this->config_.hole_min_p)
        {
          cloud_for->points.push_back(pcl::PointXYZ(this->config_.box_x_ini,threshold[cell]+(threshold[cell+1]-threshold[cell])*colIndex/4,Z));
        }
      }
    }
    Z=Z+0.05;
  }
}
