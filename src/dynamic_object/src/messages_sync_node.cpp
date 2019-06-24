/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年11月23日
* Copyright    :
* Descriptoin  : Demo for synchronizing different ROS messages
* References   :
======================================================================*/
//C/C++
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <string>
#include <iomanip>
//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
//OpenCV
#include <opencv2/opencv.hpp>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include "boost/asio.hpp"

//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "utils/messages_sync.h"
#include "utils/lidar_camera_projection_utils.h"
using namespace std;

string imagewithbboxes_topic_name = "/darknet_ros/image_with_bboxes";
string camera_topic_name = "/kitti/camera_color_left/image_raw",
       lidar_topic_name = "/kitti/velo/pointcloud";


void syncImageCloud(ros::NodeHandle nh, Eigen::MatrixXf transform_matrix)
{
  sensors_fusion::MessagesSync sync_m(nh, camera_topic_name, lidar_topic_name);
  while (ros::ok()) {
    ros::spinOnce();
    sensors_fusion::MessagesSync::SyncImageCloudPair imagePair = sync_m.getSyncMessages();
    if((imagePair.first == nullptr) || (imagePair.second == nullptr)) {
      ROS_WARN("WAIT FOR DATA");
      continue;
    }
    // get point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
//    imagePair.second->fields[3].name = "intensity";
    pcl::fromROSMsg(*imagePair.second, *tempcloud);//获取当前帧点云数据
    // get image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imagePair.first,  sensor_msgs::image_encodings::BGR8);
    cv::Mat image_raw = cv_ptr->image;
    // apply projection
    cv::Mat img_fusion_result = sensors_fusion::ProjectCloud2Image(tempcloud, image_raw, transform_matrix, true);
    cv::namedWindow("img_fusion_result", CV_WINDOW_NORMAL);
    imshow("img_fusion_result", img_fusion_result);
    cv::waitKey(5);
  }
}

void syncImageWithBBoxesCloud(ros::NodeHandle nh, Eigen::MatrixXf transform_matrix)
{
  sensors_fusion::MyMessagesSync sync_m(nh, imagewithbboxes_topic_name, lidar_topic_name);
  while (ros::ok()) {
    ros::spinOnce();
    sensors_fusion::MyMessagesSync::SyncImageCloudPair imagePair = sync_m.getSyncMessages();
    if((imagePair.first == nullptr) || (imagePair.second == nullptr)) {
      ROS_WARN("WAIT FOR DATA");
      continue;
    }
    // get point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
    pcl::fromROSMsg(*imagePair.second, *tempcloud);//获取当前帧点云数据
    // get image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(imagePair.first->image,  sensor_msgs::image_encodings::BGR8);
    cv::Mat image_raw = cv_ptr->image;
    // apply projection
    cv::Mat img_fusion_result = sensors_fusion::ProjectCloud2Image(tempcloud, image_raw, transform_matrix);
    cv::namedWindow("img_fusion_result", CV_WINDOW_NORMAL);
    imshow("img_fusion_result", img_fusion_result);
    cv::waitKey(5);
  }
}

int main(int argc,char** argv)
{
  ros::init(argc, argv, "messages_sync_node");
  ros::NodeHandle nh;


  sensors_fusion::ProjectionSingleton* projection = sensors_fusion::ProjectionSingleton::getInstance();
  projection->init();

  // transform matrix
  Eigen::MatrixXf transform_matrix = projection->getTransformMatrix();

  syncImageCloud(nh, transform_matrix);
  return 0;
}
