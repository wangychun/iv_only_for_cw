/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 点云多帧融合
* 输入：给定车体坐标系下连续点云和获取每一帧点云时车体相对于全局坐标系的位姿旋转平移矩阵
* 输出：将历史一定数量帧数点云借助全局坐标系变换到当前车体坐标系下，实现点云多帧融合
* References   :
======================================================================*/

#ifndef SRC_DYNAMICOBJECT_SRC_DENSE_POINT_CLOUD_GENERATION_H_
#define SRC_DYNAMICOBJECT_SRC_DENSE_POINT_CLOUD_GENERATION_H_

#include "transform/rigid_transform.h"
#include <ctime>
#include <iostream>
#include <math.h>
#include <sstream>
#include <vector>
#include <list>
//PCL
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/don.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
//#include "velodyne/HDL32Structure.h"
//#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
//#include "velodyne/data_types.hpp"
#include "sensor_driver_msgs/OdometrywithGps.h"
/**
 * @brief   Class for multi frame cloud fusion
 * Usage examples:
 *      DensePointCloudGeneration dense_cloud_generate_;
 *      dense_cloud_generate_.setCloudWithPose(tempcloud,timeposepair->second);
        dense_cloud_generate_.UpdateCloud();
        pcl::PointCloud<pcl::PointXYZI>::Ptr fusion_cloud = dense_cloud_generate_.getDenseCloud();
        printf("dense_cloud_generate_ size is %d\n",fusion_cloud->size());
 */
class DensePointCloudGeneration {
public:
  DensePointCloudGeneration();
  virtual ~DensePointCloudGeneration();
  //输入当前帧点云及将当前车辆位姿转变成全局坐标的变换
  void setCloudWithPose(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_get,const transform::Rigid3d& transform_get);
  void UpdateCloud();//刷新点云,即完成点云转换操作
  /*
   * get the final dense point cloud
   */
  pcl::PointCloud<pcl::PointXYZI>::Ptr getDenseCloud();
private:
  static constexpr int TotalCloudNum = 2;
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;//获取的当前帧点云(局部坐标系)
  pcl::PointCloud<pcl::PointXYZI>::Ptr dense_cloud_; //历史帧点云+当前帧融合之后的密集点云(局部坐标系)
  transform::Rigid3d current_transform_; //获得的当前帧变换
  std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_queue_;//点云历史帧(包括当前帧),保持一定数量帧,全局坐标系
  std::list<transform::Rigid3d> transform_queue_;//对应历史帧点云的位姿变换矩阵
};

#endif /* SRC_DYNAMICOBJECT_SRC_DENSE_POINT_CLOUD_GENERATION_H_ */
