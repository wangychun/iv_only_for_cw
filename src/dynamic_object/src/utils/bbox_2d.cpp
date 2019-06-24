#include "bbox_2d.h"
//C++
#include <algorithm>
#include "utils/lidar_camera_projection_utils.h"

namespace sensors_fusion {
using namespace cv;
using namespace std;
//////////////////////////////// BBox2DBase ////////////////////////////////
BBox2DBase::BBox2DBase(): bbox_(Rect(0,0,0,0)), className_(""), confidence_(1.0){}

BBox2DBase::BBox2DBase(const cv::Rect& rect, const std::string& className, float confidence):
  bbox_(rect), className_(className), confidence_(confidence)
{

}


//////////////////////////////// BBox2D ////////////////////////////////

BBox2D::~BBox2D() {
}

BBox2D::BBox2D(const Cloud cloud, const Eigen::MatrixXf& project_matrix, const cv::Size& img_size)
{
  for(const auto& point : cloud.points) {
    if(point.y < 0)
      continue;
    //1) calculate projection points
    Point2f img_point;
    sensors_fusion::ProjectPoint2Image(point, project_matrix, img_point);
    //2) get top left and bottom right points
    min_point_<< std::min(min_point_.x(), img_point.x),
        std::min(min_point_.y(), img_point.y);
    max_point_<< std::max(max_point_.x(), img_point.x),
        std::max(max_point_.y(), img_point.y);
  }
  // check rectangle validity
  CheckImageBorder(img_size);
  bbox_ = Rect(cv::Point(min_point_.x(), min_point_.y()), cv::Point(max_point_.x(), max_point_.y()));
}

BBox2D::BBox2D(const CloudPtr cloud, const Eigen::MatrixXf& project_matrix, const cv::Size& img_size)
{
  for(const auto& point : cloud->points) {
    if(point.y < 0)
      continue;
    //1) calculate projection points
    Point2f img_point;
    sensors_fusion::ProjectPoint2Image(point, project_matrix, img_point);
    //2) get top left and bottom right points
    min_point_<< std::min(min_point_.x(), img_point.x),
                 std::min(min_point_.y(), img_point.y);
    max_point_<< std::max(max_point_.x(), img_point.x),
                 std::max(max_point_.y(), img_point.y);
  }
  // check rectangle validity
  CheckImageBorder(img_size);
  bbox_ = Rect(cv::Point(min_point_.x(), min_point_.y()), cv::Point(max_point_.x(), max_point_.y()));
}

void BBox2D::CheckImageBorder(const cv::Size& img_size)
{
  if(min_point_.x() < 0)
    min_point_.x() = 0;
  if(min_point_.y() < 0)
    min_point_.y() = 0;

  if(max_point_.x() > img_size.width)
    max_point_.x() = img_size.width;
  if(max_point_.y() > img_size.height)
    max_point_.y() = img_size.height;
}

bool BBox2D::DrawBBox(cv::Mat& img_in, const cv::Scalar& color) const
{
  cv::rectangle(img_in, this->bbox_, color);
  return true;
}

} /* namespace sensors_fusion */
