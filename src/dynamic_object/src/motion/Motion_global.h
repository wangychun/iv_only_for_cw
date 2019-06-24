
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "StructMovingTargetDefine.h"
#include "TrajectoryDetection.h"
#include <visualization/ShowResult.h>
using namespace std;


class Motion_global
{
public:
  Motion_global();
  ~Motion_global();
  void Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
        const int& virtual_line_num_, const float& virtual_line_resolution_);
  void MapInput(int frame_counter_,double  time_stamp_, State_Vehicle ego_veh_state_);

  void MotionState_Filter_ab( MovingObject *moving_object_);

private:
  void FilterMethold_KF_PosHead_ab( MovingObject *moving_object_);
  //Kalman滤波更新目标中心点位置
  void FilterMethold_KF_Center2_ab( MovingObject *moving_object_);
  void FilterUpdate_Pt4_ab( MovingObject *moving_object_);
  void MotionState_Opt( MovingObject *moving_object_);

private:
  int frame_counter;
  double time_stamp;
  double time_pre_; //上一时刻时间戳
  double delta_time_;
  State_Vehicle ego_veh_state_current;

  //map
  int map_width;
  int map_height;
  int map_offset_y;
  float map_resolution;
  CvPoint  ego_veh_position;

  IplImage* img_ogm;
  IplImage* img_result;
};
