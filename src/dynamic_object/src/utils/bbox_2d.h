/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年11月5日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_BBOX_2D_H_
#define SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_BBOX_2D_H_

//C++
#include <limits>
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//Eigen
#include <Eigen/Core>
namespace sensors_fusion {
typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef Cloud::Ptr CloudPtr;

//////////////////////////////// BBox2DBase ////////////////////////////////
/**
 * @brief    Base Class to represent bounding box in camera like image
 */
class BBox2DBase
{
public:
  BBox2DBase();
  BBox2DBase(const cv::Rect& rect, const std::string& className = "", float confidence = 1.0);

  cv::Rect bbox_;

  std::string className_;

  float confidence_;
};

//////////////////////////////// BBox2D ////////////////////////////////
/**
 * @brief     Class for lidar cloud points bounding box in 2d camera view
 */
class BBox2D {
public:
  BBox2D() = default;
  BBox2D(const CloudPtr cloud, const Eigen::MatrixXf& project_matrix, const cv::Size& img_size);
  BBox2D(const Cloud cloud, const Eigen::MatrixXf& project_matrix, const cv::Size& img_size);
  virtual ~BBox2D();

  inline int area() const { return bbox_.area(); }
  inline cv::Point center() const { return cv::Point(0.5*(bbox_.tl().x + bbox_.br().x),
                                           0.5*(bbox_.tl().y + bbox_.br().y)); }
  bool DrawBBox(cv::Mat& img_in, const cv::Scalar& color = cv::Scalar(255, 0, 0)) const;
  /**
   * @brief correct the bounding box size according to actual image size
   * @param img_size[in] image size
   */
  void CheckImageBorder(const cv::Size& img_size);
private:
  cv::Rect bbox_;
  Eigen::Vector2f min_point_ = Eigen::Vector2f(std::numeric_limits<float>::max(),
                                               std::numeric_limits<float>::max());
  Eigen::Vector2f max_point_ = Eigen::Vector2f(std::numeric_limits<float>::lowest(),
          std::numeric_limits<float>::lowest());
};

} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_BBOX_2D_H_ */
