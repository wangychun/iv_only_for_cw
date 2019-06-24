// Copyright (C) 2018  Haiming Zhang<zhanghm_1995@qq.com>,
// Intelligent Vehicle Research Center, Beijing Institute of Technology

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年8月18日
 * Copyright    :
 * Descriptoin  : All functions or class definition related to lidar points and camera image projection
 *                完成激光雷达和相机之间的投影关系
 *                1、保存相机和激光雷达投影关系矩阵
 *                2、实现激光雷达点云投影到相机画面上
 *                假设点云坐标轴定义统一为: x-rightward, y-forward, z-upward
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_LIDAR_CAMERA_PROJECTION_UTILS_H_
#define SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_LIDAR_CAMERA_PROJECTION_UTILS_H_
//C++
#include <chrono>
#include <vector>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//OpenCV
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Dense>
#include "utils/bbox_2d.h"
namespace sensors_fusion {

typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr CloudConstPtr;

/*!
 * @brief given 3D point and 3x4 project matrix, this function calculate the projected point in image
 * @param pointIn            3D point coordinate
 * @param projectmatrix      3x4 project matrix
 * @param point_project[out] what we want
 * @return
 */
template <class PointT>
bool ProjectPoint2Image(const PointT& pointIn, const Eigen::MatrixXf& projectmatrix, cv::Point2f& point_project)
{
  //check project matrix size
  if(!(projectmatrix.rows()==3&&projectmatrix.cols()==4)) {
    std::cout<<"[WARN] project matrix size need be 3x4!"<<std::endl;
    return false;
  }

  //apply the projection operation
  Eigen::Vector4f point_3d(pointIn.x, pointIn.y, pointIn.z, 1.0);//define homogenious coordniate
  Eigen::Vector3f point_temp = projectmatrix*point_3d;
  //get image coordinates
  float x = static_cast<float>(point_temp[0]/point_temp[2]);
  float y = static_cast<float>(point_temp[1]/point_temp[2]);
  point_project = cv::Point2f(x,y);
  return true;
}

/*!
 * @brief project cloud points to image and use color to encode points distance
 * @param cloudIn
 * @param imageIn
 * @param projectmatrix
 * @param isKittiData[optional]  if is kitti dataset, X - forward, Y - leftward
 * @return  final image with point cloud
 */
cv::Mat ProjectCloud2Image(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudIn, const cv::Mat& imageIn, const Eigen::MatrixXf& projectmatrix, bool isKittiData = false);

/**
 * @brief project RGB cloud points to camera image, every points in cloud with the color property
 * @param cloudIn[in]
 * @param imageIn[in]
 * @param projectMatrix[in]
 * @return
 */
cv::Mat ProjectRGBCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudIn, const cv::Mat& imageIn, const Eigen::MatrixXf& projectMatrix);

#if 0
template<typename PointT>
std::vector<BBox2D> Create2DBBox(const std::shared_ptr<std::vector<pcl::PointCloud<PointT>, Eigen::aligned_allocator<pcl::PointCloud<PointT> >>> cloudVecIn, const Eigen::MatrixXf& projectMatrix, const cv::Size& imageSize)
{
  std::vector<BBox2D> bbox_vec_res;
  for(int i = 0; i < cloudVecIn->size(); ++i) {
    BBox2D bbox((*cloudVecIn)[i], projectMatrix, imageSize);
    bbox_vec_res.push_back(bbox);
  }
  return bbox_vec_res;
}
#endif

/**
 * @brief given cloud vector and project matrix, create the bounding boxes contains cloud in image
 * @param cloudVecIn
 * @param projectMatrix
 * @param imageSize
 * @return
 */
std::vector<BBox2D> Create2DBBox(const std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudVecIn, const Eigen::MatrixXf& projectMatrix, const cv::Size& imageSize);

/*!
 * Class singleton design for lidar point cloud to image projection
 */
class ProjectionSingleton
{
public:
  static ProjectionSingleton* getInstance();

  void init(Eigen::MatrixXf transform_matrix = Eigen::MatrixXf::Zero(3,4));
  const Eigen::MatrixXf& getTransformMatrix() const { return transform_matrix_; }
  void setTransformMatrix(Eigen::MatrixXf transform_matrix) { transform_matrix_ = transform_matrix; }
private:
  ProjectionSingleton(){};
  virtual ~ProjectionSingleton(){}
  ProjectionSingleton(const ProjectionSingleton& rhs);
  ProjectionSingleton& operator=(const ProjectionSingleton& rhs);

private:
  static ProjectionSingleton* uniqueInstance;

  Eigen::MatrixXf transform_matrix_;
};

/*!
 * @brief from bounding boxes to extract all
 * @param bboxes
 * @param cloudIn
 * @param transform_matrix
 * @return
 */
std::vector<pcl::PointCloud<pcl::PointXYZI> > ExtractCloudFromBBoxes(const std::vector<BBox2DBase>& bboxes, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Eigen::MatrixXf transform_matrix);


/*!
 * @brief  Old class for projecting HDL 64 cloud to camera image
 */
class LidarCameraProjectionUtils {
public:
  //constructors
  LidarCameraProjectionUtils();
  LidarCameraProjectionUtils(const Eigen::MatrixXd& Rmat, const Eigen::MatrixXd& tmat, const Eigen::MatrixXd& instrinsicA);
  virtual ~LidarCameraProjectionUtils();

  //投影单个点
  //just complete matrix multiplication, no validity check
  cv::Point ProjectPoint2Image(const pcl::PointXYZI& pointIn);

  void DrawProjectedCloud(CloudConstPtr lidarCloudIn, cv::Mat& img_src);

private:
  Eigen::Matrix3d Rmat_;//旋转矩阵
  Eigen::Vector3d tmat_;//平移矩阵
  Eigen::Matrix3d intrinsicA_;//相机内参
};

} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_LIDAR_CAMERA_PROJECTION_UTILS_H_ */
