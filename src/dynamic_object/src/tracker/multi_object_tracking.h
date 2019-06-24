/*======================================================================
* Author   : Haiming Zhang
* Email    :
* Maintainer:
* Version  :
* Copyright    :
* Descriptoin  : 目标跟踪主功能类，与外界接口，使用其他功能类，共同完成
*                1、点云栅格地图生成
*                2、目标检测
*                3、目标跟踪
*                等不同任务
* References   :
======================================================================*/

#ifndef SRC_DYNAMIC_OBJECT_SRC_MULTI_OBJECT_TRACKING_H_
#define SRC_DYNAMIC_OBJECT_SRC_MULTI_OBJECT_TRACKING_H_

#include <array>
#include <vector>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h>

class MultiObjectTracking {
public:
  MultiObjectTracking();
  virtual ~MultiObjectTracking();
  void SetCloudInput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in){cloud_input_ = cloud_in;}//原始点云输入
  void ProcessFrame();//对外接口,主处理函数

  pcl::PointCloud<pcl::PointXYZ>::Ptr getElevatedCloud(){return elevatedCloud_;}
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getClusteredCloud(){return clusteredCloud_;}
  std::vector<pcl::PointCloud<pcl::PointXYZ>> getBoxes(){return bBoxes_;}
private:
  void PreProcess();//点云预处理,包括地面点云移除,点云聚类

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input_;//原始输入点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr elevatedCloud_;//高程点
  pcl::PointCloud<pcl::PointXYZ>::Ptr groundCloud_;//地面点云
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>> bBoxes_;
};

#endif /* SRC_DYNAMIC_OBJECT_SRC_MULTI_OBJECT_TRACKING_H_ */
