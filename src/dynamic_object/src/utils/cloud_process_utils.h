/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年8月21日
 * Copyright    :
 * Descriptoin  : Cloud process utilities functions, including
 *                cloud preprocess(filter),
 *                cloud transformation, cloud basic handling etc.
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_UTILS_CLOUD_PROCESS_UTILS_H_
#define SRC_DYNAMIC_OBJECT_SRC_UTILS_CLOUD_PROCESS_UTILS_H_
//C++
#include <cmath>
#include <map>
#include <string>
//OpenCV
#include <opencv/cv.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
//Eigen
#include <Eigen/Dense>
#include "StructMovingTargetDefine.h"
namespace sensors_fusion {

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;

template<typename PointT>
using CloudPair = std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ;
/*!
 * @brief according to distance limit to filter cloud and remove points on ego vehicle at the same time
 * @param x_min meter
 * @param x_max
 * @param y_min
 * @param y_max
 * @param z_min
 * @param z_max
 * @return
 */
inline CloudPtr CloudFilter(const CloudPtr cloudIn, float x_min,float x_max, float y_min, float y_max, float z_min, float z_max)
{
  //1) set parameters for removing cloud reflect on ego vehicle
  float x_limit_min = -1.8, x_limit_max = 1.8,
        y_limit_forward = 5.0, y_limit_backward = -4.5;
  //2 apply the filter
  CloudPtr filteredCloud(new Cloud());
  for(int i = 0; i < cloudIn->size(); ++i){
    float x = cloudIn->points[i].x;
    float y = cloudIn->points[i].y;
    float z = cloudIn->points[i].z;
    // whether on ego vehicle
    if((x > x_limit_min && x < x_limit_max && y > y_limit_backward && y < y_limit_forward))
      continue;
    if((x > x_min && x < x_max && y > y_min && y < y_max && z > z_min && z < z_max)) {
      filteredCloud->points.push_back(cloudIn->points[i]);
    }
  }
  return filteredCloud;
}

template <typename PointInT, typename PointOutT>
void CloudFilter(const pcl::PointCloud<PointInT>& cloudIn, pcl::PointCloud<PointOutT>& cloudOut,
                 float x_min,float x_max, float y_min, float y_max, float z_min, float z_max)
{
  cloudOut.header = cloudIn.header;
  cloudOut.sensor_orientation_ = cloudIn.sensor_orientation_;
  cloudOut.sensor_origin_ = cloudIn.sensor_origin_;
  cloudOut.points.clear();
  //1) set parameters for removing cloud reflect on ego vehicle
  float x_limit_min = -1.8, x_limit_max = 1.8,
        y_limit_forward = 5.0, y_limit_backward = -4.5;
  //2 apply the filter
  for(int i = 0; i < cloudIn.size(); ++i){
    float x = cloudIn.points[i].x;
    float y = cloudIn.points[i].y;
    float z = cloudIn.points[i].z;
    // whether on ego vehicle
    if((x > x_limit_min && x < x_limit_max && y > y_limit_backward && y < y_limit_forward))
      continue;
    if((x > x_min && x < x_max && y > y_min && y < y_max && z > z_min && z < z_max)) {
      cloudOut.points.push_back(cloudIn.points[i]);
    }
  }
}

/*!
 * @brief transform Kitti dataset cloud point to common vehicle coordinate
 * @param kitti_cloud x-forward, y-left z-upward
 * @param do_z_shift  whether apply Z direction shift for obtain z value wrt. ground
 * @return  common vehicle coordinate---x-right  y-forward  z-upward
 */
template <typename PointT>
void TransformKittiCloud(const pcl::PointCloud<PointT>& kitti_cloud, pcl::PointCloud<PointT>& cloudOut, bool do_z_shift = false, float z_shift_value = 1.73)
{
  //do transformation
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  // Define translation
  if(do_z_shift)
    transform_matrix.translation() << 0.0, 0.0, z_shift_value;
  else
    transform_matrix.translation() << 0.0, 0.0, 0.0;
  // The same rotation matrix as before; theta radians around Z axis
  transform_matrix.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));

  // Executing the transformation
  pcl::transformPointCloud (kitti_cloud, cloudOut, transform_matrix);
}

/**
 * @brief assign specific color for input Point Cloud
 * @param cloudIn[in]
 * @param cloudOut[out]  output PointXYZRGB type Point Cloud
 * @param color
 */
template <typename PointT>
void AssignCloudColor(const typename pcl::PointCloud<PointT>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut, cv::Scalar color = cv::Scalar(0, 255, 0))
{
  pcl::copyPointCloud(*cloudIn, *cloudOut);
  for(auto& p:cloudOut->points) {
    p.r = color.val[3]; p.g = color.val[2]; p.b = color.val[0];
  }
}

//Input: 点云
//Output: 选出最可靠聚类,输出该聚类离原点平均距离
template <typename PointT>
float calculateAverageDistance(const pcl::PointCloud<PointT>& cloud)
{
  float totalDistance = 0.0;
  for(int i = 0; i < cloud.size(); ++i) {
    float x = cloud.points[i].x;
    float y = cloud.points[i].y;
    float z = cloud.points[i].z;
    totalDistance += sqrt(x*x + y*y + z*z);
  }
  return totalDistance/cloud.size();
}

//TODO: below code snippet cannot compile ok, don't know the reason
#if 0
template <typename PointT>
CloudPair<PointT> RemoveGround(const typename pcl::PointCloud<PointT>::Ptr cloudIn, const OGMProperty& ogm_property, vector<OGM_Cell>& ogm_cells_vec, float heightTh)
{
  float ground_threshold = 0.15; //障碍物栅格中离最小z值以上0.15m以内认为是地面
  PointT cloud_no_ground, cloud_ground;//障碍物点云,地面点云
  for(int i = 0; i < cloudIn->size();++i) {
    float x = cloudIn->points[i].x;
    float y = cloudIn->points[i].y;
    float z = cloudIn->points[i].z;

    float newy = y + ogm_property.ogm_y_offset_;
    if (x >= -ogm_property.ogm_width_/2 && x < ogm_property.ogm_width_/2  && newy >= 0 && newy < ogm_property.ogm_height_) {
      int col = (int)((x + ogm_property.ogm_width_/2) / ogm_property.resolution_);
      int row = (int)(newy / ogm_property.resolution_) ;
      int index = row * ogm_property.mapSize().width + col;//栅格地图数组中索引
      if(ogm_cells_vec[index].delta_z <= heightTh) {//地面点
        ogm_cells_vec[index].type = CellProperty::Ground;
        cloud_ground.push_back(cloudIn->points[i]);
        continue;
      }
      else {//认为是障碍物点
        //        if(ogm_cells_vec[index].points_num>10)
        if(ogm_cells_vec[index].points_num > 5) {
          ogm_cells_vec[index].type = CellProperty::RigidNoPassable;//障碍物栅格
          if(z > (ogm_cells_vec[index].min_z + ground_threshold)){//去除掉其中的部分地面点云
            cloud_no_ground.push_back(cloudIn->points[i]);
            int cloud_idx = (int)cloud_no_ground.size() - 1;
            ogm_cells_vec[index].cloud_index.push_back(cloud_idx);//得到障碍物栅格对应的点云索引
          }
        }
      }
    }
  }//for(int i = 0; i < filterCloud->size();++i)
  return CloudPair<PointT>(cloud_no_ground.makeShared(), cloud_ground.makeShared());
}
#endif

CloudPair<pcl::PointXYZI> RemoveGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, const OGMProperty& ogm_property, vector<OGM_Cell>& ogm_cells_vec, float heightTh);


template <typename PointT>
void RemoveGround(const pcl::PointCloud<PointT>& cloudIn, pcl::PointIndices::Ptr& indices, const OGMProperty& ogm_property, vector<OGM_Cell>& ogm_cells_vec, float heightTh)
{
  float ground_threshold = 0.15; //障碍物栅格中离最小z值以上0.15m以内认为是地面
  for(int i = 0; i < cloudIn.size(); ++i) {
    float x = cloudIn.points[i].x;
    float y = cloudIn.points[i].y;
    float z = cloudIn.points[i].z;

    float newy = y + ogm_property.ogm_y_offset_;
    if (x >= -ogm_property.ogm_width_/2 && x < ogm_property.ogm_width_/2  && newy >= 0 && newy < ogm_property.ogm_height_) {
      int col = (int)((x + ogm_property.ogm_width_/2) / ogm_property.resolution_);
      int row = (int)(newy / ogm_property.resolution_) ;
      int index = row * ogm_property.mapSize().width + col;//栅格地图数组中索引
      if(ogm_cells_vec[index].delta_z <= heightTh) {//地面点
        ogm_cells_vec[index].type = CellProperty::Ground;
        continue;
      }
      else {//认为可能是障碍物栅格
        if(ogm_cells_vec[index].points_num > 5) {
          ogm_cells_vec[index].type = CellProperty::RigidNoPassable;
          if(z > (ogm_cells_vec[index].min_z + ground_threshold)) {//去除掉其中的部分地面点云
            indices->indices.push_back(i);
            int cloud_idx = (int)indices->indices.size() - 1;
            ogm_cells_vec[index].cloud_index.push_back(cloud_idx);//得到障碍物栅格对应的点云索引
          }
        }
      }
    }
  }//for(int i = 0; i < filterCloud->size();++i)
}

/*!
 * @brief ExtrinsicTransformParams class
 *        transfrom the cloud point in lidar coordinate to vehicle coordinate
 */
class ExtrinsicTransformParams
{
public:
  ExtrinsicTransformParams(const Eigen::Matrix4f& transform_matrix_calibration):transform_matrix_calibration_(transform_matrix_calibration){ }

  ExtrinsicTransformParams(float alpha, float beta, float gama, float x_offset, float y_offset, float z_offset){
    transform_matrix_calibration_(0,0) = cos(beta)*cos(gama) - sin(beta)*sin(alpha)*sin(gama);
    transform_matrix_calibration_(1,0) = cos(beta)*sin(gama) + sin(beta)*sin(alpha)*cos(gama);
    transform_matrix_calibration_(2,0) = -cos(alpha)*sin(beta);

    transform_matrix_calibration_(0,1) = -cos(alpha)*sin(gama);
    transform_matrix_calibration_(1,1) = cos(alpha)*cos(gama);
    transform_matrix_calibration_(2,1) = sin(alpha);

    transform_matrix_calibration_(0,2) = sin(beta)*cos(gama) + cos(beta)*sin(alpha)*sin(gama);
    transform_matrix_calibration_(1,2) = sin(beta)*sin(gama) - cos(beta)*sin(alpha)*cos(gama);
    transform_matrix_calibration_(2,2) = cos(alpha)*cos(beta);

    transform_matrix_calibration_(3,0) = 0;
    transform_matrix_calibration_(3,1) = 0;
    transform_matrix_calibration_(3,2) = 0;
    transform_matrix_calibration_(3,3) = 1.0;

    transform_matrix_calibration_(0,3) = x_offset;
    transform_matrix_calibration_(1,3) = y_offset;
    transform_matrix_calibration_(2,3) = z_offset;
  }

  Eigen::Matrix4f get_transform_matrix() const { return transform_matrix_calibration_; }

private:
  Eigen::Matrix4f transform_matrix_calibration_ =  Eigen::Matrix4f::Identity();
};

class CloudTransformUtils {
public:
  CloudTransformUtils();
  virtual ~CloudTransformUtils();

  void set_extrinsic_params(std::string lidarName, ExtrinsicTransformParams extrinsic_params) { this->extrinsic_params_.insert(std::make_pair(lidarName, extrinsic_params)); }
  void ApplyTransform(std::string lidarName, CloudPtr cloudIn, CloudPtr cloudOut);
  std::map<std::string, ExtrinsicTransformParams> extrinsic_params_;
};

} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_UTILS_CLOUD_PROCESS_UTILS_H_ */
