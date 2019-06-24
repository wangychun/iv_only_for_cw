/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月1日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#ifndef SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_DETECTION_H_
#define SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_DETECTION_H_
//Includes
//C++
#include <iostream>
#include <vector>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//OpenCV
#include <opencv2/opencv.hpp>
//Eigen
#include <Eigen/Dense>
//Project
#include <utils/lidar_camera_projection_utils.h>
#include <utils/cloud_process_utils.h>
#include <utils/messages_sync.h>
#include <detect/ogm_detector.h>
#include <visualization/visualizer_utils.h>
#include <utils/cloud_process_utils.h>
#include <utils/ogm_mapping_utils.h>
#include <utils/bbox_2d.h>
#include <darknet_ros_msgs/ImageWithBBoxes.h>

#include <visualization/visualization.h>
#include <sensors_fusion/fusion_type.h>

// Types of point and cloud to work with
typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointXYZRGB VRGBPoint;
typedef pcl::PointCloud<VRGBPoint> VRGBPointCloud;

namespace sensors_fusion {

std::map<std::string, cv::Scalar> color ={ {"Car", cv::Scalar(142,0,0)},
                                           {"Pedestrian", cv::Scalar(60,20,220)},
                                           {"Cyclist", cv::Scalar(32, 11, 119)} };

class FusionDetection {
public:
  // Default constructor
  FusionDetection(ros::NodeHandle nh, ros::NodeHandle private_nh);
  virtual ~FusionDetection();
  // main loop function for handling all detection related works
  void process();

private:
  void processPointCloud(const sensor_msgs::PointCloud2::ConstPtr & cloud, bool isShow = true);
  void processImage(const darknet_ros_msgs::ImageWithBBoxes::ConstPtr& image);
  //! from image detection results extract cloud clusters
  void processCluster(VPointCloud::Ptr cloudIn);

  void removeClustersBackground();

  void boxFitting();

  vector<sensors_fusion::BBox2DBase> darknetMsgToBBoxes(const darknet_ros_msgs::ImageWithBBoxesConstPtr& darknetMsg);

  void drawBBoxes(cv::Mat& image, const vector<sensors_fusion::BBox2DBase>& bbox);

private:

  // Node handle
  ros::NodeHandle nh_, private_nh_;

  // Class members
  VPointCloud::Ptr cloud_raw_;
  VPointCloud::Ptr cloud_limit_; // filtering cloud to get what we interested
  VPointCloud::Ptr cloud_elevated_;
  VPointCloud::Ptr cloud_ground_;
  VPointCloud::Ptr cloud_camera_elevated_;

  // OGM property
  OGMProperty ogm_property_;

  // Image
  cv::Mat image_raw_;

  // Publisher
  ros::Publisher cloud_limit_pub_;
  ros::Publisher cloud_elevated_pub_;
  ros::Publisher cloud_camera_elevate_pub_;
  ros::Publisher cloud_clustered_rgb_pub_;
  ros::Publisher image_detection_pub_;

  // Class member
  std::vector<ObjectTrack> clusters_track_;
  std::vector<std::vector<pcl::PointIndices> > cluster_indices_;

  dynamic_object_tracking::VisualizerUtils* vis_result_;
  visualization::Visualization rviz_vis_;

  std::vector<OGM_Cell> ogm_cells_vector_;

  // Image and cloud synchronizer
  sensors_fusion::MyMessagesSync* imageCloudSync_;

  //! detected bounding boxes in input image
  std::vector<sensors_fusion::BBox2DBase> detectBBoxes_;

  // Multi thread
  boost::thread* processthread_;
  bool processthreadfinished_;

  // Init counter
  int frame_count_;

  sensors_fusion::ProjectionSingleton* projection_tools_;

};

} /* namespace sensors_fusion */

#endif /* SRC_DYNAMIC_OBJECT_SRC_SENSORS_FUSION_FUSION_DETECTION_H_ */
