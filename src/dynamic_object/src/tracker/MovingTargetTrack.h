/*======================================================================
 * Author   : Weijie Meijie
 * Email    :
 * Maintainer:  Haiming Zhang
 * Version  :
 * Copyright    :
 * Descriptoin  : 目标跟踪主功能类，与外界接口，使用其他功能类，共同完成
 *                1、点云栅格地图生成
 *                2、目标检测
 *                3、目标跟踪
 *                等不同任务
 * References   :
======================================================================*/
#pragma once
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <typeinfo>
#include <deque>

#include <opencv2/opencv.hpp>

#include "utils/cloud_process_utils.h"
#include "detect/ogm_detector.h"
#include "utils/ogm_mapping_utils.h"
#include "StructMovingTargetDefine.h"

#include "motion/TargetTracking.h"
#include "multi_object_tracking.h"//TODO,应该以一个类为主

#include "iv_dynamicobject_msgs/DynamicMap.h"
#include "match/TargetMatching.h"
#include <visualization/visualizer_utils.h>
#include <visualization/ShowResult.h>

class MovingTargetTrack
{
public:
  typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;
  //constructors
  MovingTargetTrack(const OGMProperty& ogm_property, const PolarProperty& polar_property);
  ~MovingTargetTrack();

  bool Init(bool is_show_submodule_result = true);
  void SetBasicInput(double time_stamp,const State_Vehicle& ego_veh_state_current, bool _is_open_dynamic_object = true);

  void ProcessFrame();//!主处理函数:处理原始点云
  void ProcessMapFrame();//!直接处理已经预处理过的点云栅格图

  void TargetDetection();//目标检测
  void MultiTargetMatch();//当前帧检测结果和跟踪上目标进行匹配

  /************************************
   * zhanghm: add 20180525
   **********************************/
  void SetCloudInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in);
  void DrawResult();
  // Show final tracking results
  void Show_Result_2D(double elapsed_time = 0.0);

  /// -------------get and set functions---------------------
  MovingTargetSend get_Moving_Target_Send() const{return this->moving_target_send;}
  /*!
   * @brief Get current single frame grid map for lateral motion objects to Planning Module
   * @param isRemove Whether remove motion objects occupied cells
   * @return
   */
  cv::Mat getMovingTargetMap(bool& isLateralMoving, bool isRemoveTarget = true, bool isFillMap = false);
  const MovingTargetOutput get_moving_obj_send() const { return this->moving_target_output_; }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRGBClusteredCloud(){return target_detector.getRGBClusteredCloud();}
  CloudPtr get_elevated_cloud_ptr(){return this->elevated_cloud_ptr_;}
  std::shared_ptr<std::vector<Cloud>> get_clustered_cloud_vec() {return this->target_detector.get_clustered_cloud_vec(); };

private:
  //私有赋值运算符函数,不允许赋值操作
  MovingTargetTrack& operator=(const MovingTargetTrack& rhs);
  //私有拷贝构造函数,不允许拷贝构造
  MovingTargetTrack(const MovingTargetTrack& rhs);

  void CreateObstacleOGM(const std::vector<OGM_Cell>& ogm_cell_vec);
  void DetermineTargetSend();
  void DetermineTargetSend(vector<MovingObject>& moving_target_vector);

  void ChooseSendTargets(const vector<MovingObject>& moving_object_send_vector);

private:
  int frame_counter;
  double total_cost_time;//每一帧总处理时间
  double time_stamp;//当前数据帧时间戳
  bool is_show_submodule_result_;
  State_Vehicle ego_veh_state_current;//车辆位姿
  OGMProperty ogm_property_;
  PolarProperty polar_property_;


  /// Cloud variables definition
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_limit_;//感兴趣原始点云,移除打在本车上的点云,并限定了感兴趣范围
  /************************************
   * zhanghm: add 20180525
   **********************************/
  pcl::PointCloud<pcl::PointXYZI> elevated_cloud_;//去除地面的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr elevated_cloud_ptr_;//去除地面的点云
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr_;

  vector<CandidateObject> fusion_object_vector;
  vector<MovingObject> moving_object_vector;//跟踪目标序列

  //OGM generate
  //栅格地图相关属性
  float refineogm_width;//Unit: m
  float refineogm_height;
  float refineogm_offset_y;
  float refineogm_resolution;
  int refineogmwidth_cell;
  int refineogmheight_cell;
  int refineogmcell_size;


  /// OGM map vector for storing ogm statistic information
  vector<OGM_Cell> rigid_refined_ogm_vec_;

  /// Polar OGM map vector for storing polar ogm statistic information
  vector<Polar_Cell> polar_cells_vector_;

  std::vector<Line_int> virtual_line_vec_;

  float virtual_line_resolution;
  int virtual_line_num;

  int map_offset_y;
  float map_resolution;

  //栅格地图
  IplImage* img_ogm;//!refineogm地图,单通道
  cv::Mat img_target_fusion_;//最终融合图像,考虑到毫米波雷达检测距离可能较远,该图像在车前多加了100个像素

  CvPoint ego_veh_position;//本车在栅格地图中的位置

  //功能类
  OGMDetector target_detector;
  TargetMatching multi_target_match;
  TrackingUpdate target_motion_updating_;
  ShowResult show_result;

  MultiObjectTracking multi_track_;

  dynamic_object_tracking::VisualizerUtils* vis_result_;
  std::deque<State_Vehicle> ego_veh_state_history_;//本车历史轨迹

  //!Target send
  vector<MovingObject> moving_object_vector_send;//发送给规划的跟踪目标向量,也是最终绘制的目标
  MovingTargetSend moving_target_send;//最终要发送的目标
  MovingTargetOutput moving_target_output_;

  bool last_lateral_moving_target_;
  int lateral_counter_;
};
