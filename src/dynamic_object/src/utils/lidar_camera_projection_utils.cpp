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

#include "lidar_camera_projection_utils.h"
#include <cmath>
using namespace cv;
using std::cout;
using std::endl;
using std::vector;
namespace sensors_fusion {

cv::Mat ProjectCloud2Image(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudIn, const cv::Mat& imageIn, const Eigen::MatrixXf& projectmatrix, bool isKittiData)
{
  cv::Mat hsv_image, res_image;
  cv::cvtColor(imageIn, hsv_image, CV_BGR2HSV);
  int scale = 120, min_dis = 1, max_dis = 70;
  auto toColor = [&](const pcl::PointXYZI& point)->int{
    //1)calculate point distance
    float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    //2)calculate color value, normalize values to (0 - scale) & close distance value has low value.
    int res = ((distance - min_dis) / (max_dis - min_dis))*scale;
    return res;
  };
  // plot color points using distance encode HSV color
  for(int i = 0;i<cloudIn->size();++i) {
    if(isKittiData) {
      if(cloudIn->points[i].x < 0)
        continue;
    }
    else {
      if(cloudIn->points[i].y < 0)
        continue;
    }
    int color = toColor(cloudIn->points[i]);
    cv::Point2f image_point;
    ProjectPoint2Image(cloudIn->points[i],projectmatrix, image_point);
    cv::circle(hsv_image, image_point, 2, Scalar(color, 255, 255), -1);
  }
  cv::cvtColor(hsv_image, res_image,CV_HSV2BGR);
  return res_image;
}

cv::Mat ProjectRGBCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudIn, const cv::Mat& imageIn, const Eigen::MatrixXf& projectMatrix)
{
  Mat res_image = imageIn.clone();
  for(int i = 0; i < cloudIn->size(); ++i) {
    if(cloudIn->points[i].y < 0)
      continue;
    Scalar color(cloudIn->points[i].b, cloudIn->points[i].g, cloudIn->points[i].r);
    cv::Point2f image_point;
    ProjectPoint2Image(cloudIn->points[i],projectMatrix,image_point);
    cv::circle(res_image, image_point, 2,color, -1);
  }
  return res_image;
}

std::vector<BBox2D> Create2DBBox(const std::vector<pcl::PointCloud<pcl::PointXYZI>> cloudVecIn, const Eigen::MatrixXf& projectMatrix, const cv::Size& imageSize)
{
  std::vector<BBox2D> bbox_vec_res;
  for(int i = 0; i < cloudVecIn.size(); ++i) {
    BBox2D bbox(cloudVecIn[i], projectMatrix, imageSize);
    bbox_vec_res.push_back(bbox);
  }
  return bbox_vec_res;
}

///////////////////////////////////////ProjectionSingleton/////////////////////////////////////////
ProjectionSingleton* ProjectionSingleton::uniqueInstance = new ProjectionSingleton();

ProjectionSingleton* ProjectionSingleton::getInstance()
{
  return uniqueInstance;
}

void ProjectionSingleton::init(Eigen::MatrixXf transform_matrix)
{
  if(!transform_matrix.isZero())
    transform_matrix_ = transform_matrix;
  else {
    Eigen::Matrix4f RT_velo_to_cam;
    Eigen::Matrix4f R_rect_00;
    Eigen::MatrixXf project_matrix(3,4);
    RT_velo_to_cam<<7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03,
                      1.480249e-02,7.280733e-04,-9.998902e-01, -7.631618e-02,
                      9.998621e-01,7.523790e-03,1.480755e-02,  -2.717806e-01,
                      0.0, 0.0, 0.0, 1.0;
    R_rect_00<<9.999239e-01,9.837760e-03,-7.445048e-03, 0.0,
                -9.869795e-03,9.999421e-01,-4.278459e-03, 0.0,
                 7.402527e-03,4.351614e-03,9.999631e-01, 0.0,
                 0.0, 0.0, 0.0, 1.0;
    project_matrix<<7.215377e+02,0.000000e+00,6.095593e+02,4.485728e+01,
                    0.000000e+00,7.215377e+02,1.728540e+02,2.163791e-01,
                    0.000000e+00,0.000000e+00,1.000000e+00,2.745884e-03;
    transform_matrix_ = project_matrix*R_rect_00*RT_velo_to_cam;
  }
}

std::vector<pcl::PointCloud<pcl::PointXYZI> > ExtractCloudFromBBoxes(const std::vector<BBox2DBase>& bboxes, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, Eigen::MatrixXf transform_matrix)
{
  std::vector<pcl::PointCloud<pcl::PointXYZI> > cloudVec(bboxes.size());
  //1) loop for every point in cloud
  for(const auto& point : *cloudIn) {
    if(point.y < 0)
      continue;
    cv::Point2f imgPoint;
    ProjectPoint2Image(point, transform_matrix, imgPoint);
    //2) loop every bbox in input bboxes
    for(int i = 0; i < bboxes.size(); ++i) {
      if(bboxes[i].bbox_.contains(imgPoint)) {
        cloudVec[i].push_back(point); // push back point
      }
    }
  }
  return cloudVec;
}


/////////////////////////////////////////////////////////////////////////////////////////////////
LidarCameraProjectionUtils::LidarCameraProjectionUtils() {

}

LidarCameraProjectionUtils::LidarCameraProjectionUtils(const Eigen::MatrixXd& Rmat, const Eigen::MatrixXd& tmat, const Eigen::MatrixXd& instrinsicA):
Rmat_(Rmat),
tmat_(tmat),
intrinsicA_(instrinsicA)
{
  cout<<Rmat_<<endl;
  cout<<tmat_<<endl;
  cout<<intrinsicA_<<endl;
}

LidarCameraProjectionUtils::~LidarCameraProjectionUtils() {
  // TODO Auto-generated destructor stub
}

cv::Point LidarCameraProjectionUtils::ProjectPoint2Image(const pcl::PointXYZI& pointIn)
{
  Eigen::Vector3d point_3d;
  point_3d << pointIn.x, pointIn.y, pointIn.z;
  //calculate projection
  Eigen::Vector3d point_t3d = Rmat_ * point_3d + tmat_;
  Eigen::Vector3d point_tt3d = intrinsicA_ * point_t3d;

  int x = static_cast<int>( (point_tt3d[0]/point_tt3d[2]+2+16.8868)/0.6848 );  //calibration number, run showLaserCameraImgCoordRes64.m ( 0.6848-xmls,  16.8868-xml1, 12.6651-xml2)
  int y = static_cast<int>( (point_tt3d[1]/point_tt3d[2]+2+12.6651)/0.6848 );
  return Point(x, y);
}

void LidarCameraProjectionUtils::DrawProjectedCloud(CloudConstPtr lidarCloudIn, cv::Mat& img_src)
{
  for(size_t i = 0; i<lidarCloudIn->points.size();++i) //scan all point cloud
  {
    //Important!! eliminate the point behind camera
    //otherwise, it would project redundant points
    if(lidarCloudIn->points[i].y < 0)
      continue;
    Point img_point = this->ProjectPoint2Image(lidarCloudIn->points[i]);
    if(img_point.x >=0 && img_point.x < img_src.cols && img_point.y > 0 && img_point.y < img_src.rows)
      circle(img_src, img_point, 2, Scalar(0, 0, 255));//绘制红色圆点
  }
}

} /* namespace sensors_fusion */
