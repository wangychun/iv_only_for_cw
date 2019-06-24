/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年7月18日
 * Copyright    :
 * Descriptoin  : singleton design patterns for visualizing occupied grid map related
 *                what we want by using the same parameters
 *
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_VISUALIZER_UTILS_H_
#define SRC_DYNAMIC_OBJECT_SRC_VISUALIZER_UTILS_H_

#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "StructMovingTargetDefine.h"

namespace dynamic_object_tracking {
//绘制本车用
const int EGO_VEHICLE_WIDTH = 2;//2m
const int EGO_VEHICLE_LENGTH = 5;//5m

class VisualizerUtils {
public:
  static VisualizerUtils* getInstance();

  void Init(float ogm_width,float ogm_height,float ogm_offset_y,float ogm_resolution,
            int map_width,int map_height,int map_offset_y);

  void Init(const OGMProperty& ogm_property);
  /*!
   * @brief 给定物理坐标系点坐标,得到栅格地图坐标
   * @param xv[in] 车体坐标系点
   * @param yv[in]
   * @param row[out] 图像坐标系点
   * @param col[out]
   * @return
   */
  bool get_image_coord(float xv,float yv,int& row,int& col);

  void DrawEgoVehicle(cv::Mat& img);
  /**
   * @brief given cloud points, this function draw the grid map
   * @param cloud_in
   * @param draw_vehicle
   * @param color
   * @return
   */
  cv::Mat DrawPointCloudOGM(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,bool draw_vehicle = true,cv::Scalar color = cv::Scalar(0,255,0));//绘制点云栅格地图

  template <typename PointT>
  cv::Mat DrawPointCloudOGM(pcl::PointCloud<PointT> cloud_in,bool draw_vehicle = true,cv::Scalar color = cv::Scalar(0,255,0))
  {
    cv::Mat img_res = cv::Mat::zeros(map_height_,map_width_,CV_8UC3);
    if(draw_vehicle)
      this->DrawEgoVehicle(img_res);
    for(int i = 0;i<cloud_in.size();++i)
    {
      //获得每个点的属性
      float x = cloud_in.points[i].x;
      float y = cloud_in.points[i].y;
      float z = cloud_in.points[i].z;
      //计算在图像位置
      int row,col;
      if(!get_image_coord(x,y,row,col))
        continue;
      img_res.at<cv::Vec3b>(row,col) = cv::Vec3b(color(0),color(1),color(2));//BGR color
    }
    return img_res;
  }

  //绘制车体坐标系下坐标点
  void DrawEgoPoints(cv::Mat& img, float x, float y, cv::Scalar color = cv::Scalar(0,255,0));
  void ShowIplImage(std::string window_name,const IplImage* img_src,bool copyData = false, bool draw_vehicle = false);
  void ShowMovingObjectVector(IplImage* img_, vector<MovingObject>* moving_object_vector);

  /**!
   * @brief Display text information in image
   * @param img
   * @param frame_counter
   * @param target_num
   * @param cost_time
   * @param ego_speed
   */
  void ShowText(cv::Mat& img, long long frame_counter,
                int target_num, float cost_time, float ego_speed);
private:
  VisualizerUtils(){};
  virtual ~VisualizerUtils(){};
  VisualizerUtils(const VisualizerUtils& rhs);
  VisualizerUtils& operator=(const VisualizerUtils& rhs);

private:
  static VisualizerUtils* uniqueInstance;
  //米制单位地图属性
  float ogm_width_;
  float ogm_height_;
  float ogm_offset_y_;
  float ogm_resolution_;
  //栅格地图图像属性(像素单位)
  int map_width_;
  int map_height_;
  int map_offset_y_;

  CvPoint ego_veh_position;//本车在图像中位置,像素坐标
  State_Vehicle ego_veh_state_current;
  int frame_counter;

  double ego_veh_dlng;
  double ego_veh_dlat;
  double ego_veh_dheading;
  float ego_veh_fForwardVel;
  double ego_veh_fFLRWheelAverAngle;
};

} /* namespace dynamic_object_tracking */

#endif /* SRC_DYNAMIC_OBJECT_SRC_VISUALIZER_UTILS_H_ */
