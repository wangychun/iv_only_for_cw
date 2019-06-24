/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年12月13日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_TYPE_H_
#define SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_TYPE_H_
//C++
#include <string>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//OpenCV
#include <opencv2/opencv.hpp>
//ROS
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>

namespace sensors_fusion {

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZRGB VRGBPoint;
typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

struct ObjectTrack
{
  std::string id;
  std::string object_type;

  float length, width, height;

  // Geometry pose info in lidar coordinate
  geometry_msgs::Pose velo_pose;

  // Extracted lidar points in this object
  // NOTE: The color encode according to CityScape dataset for every object type
  VPointCloud lidar_points;

  VPointCloud bbox3D; // 3d bboxes, contains 8 vertices

  // Bounding box in camera image
  cv::Rect bbox;

  bool is_valid;

  inline bool empty () const { return lidar_points.empty (); }
};

}// end namespace sensors_fusion

#endif /* SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_TYPE_H_ */
