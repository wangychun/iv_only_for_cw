#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include "StructMovingTargetDefine.h"
#include <visualization/ShowResult.h>
class Traj_detect
{
public:
  Traj_detect();
  ~Traj_detect();
  void Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
      const int& virtual_line_num_, const float& virtual_line_resolution_);
  void MapInput(const int& frame_counter_,const double&  time_stamp_, const State_Vehicle& ego_veh_state_);

  void UpdateMotionBehavior(MovingObject *moving_object_);

private:
  void Motion_traj_fit( MovingObject *moving_object_ , int history_num_, CvPoint2D32f* motion_traj_, float* traj_time_ );
  int Motion_behavior( MovingObject *moving_object_, int history_num_, CvPoint2D32f* motion_traj_);
  void Motion_traj( MovingObject *moving_object_ , State_Vehicle ego_veh_state_current_, double time_stamp_);
  void MotionTraj( MovingObject *moving_object_, State_Vehicle ego_veh_state_current_,double time_stamp_ );
  //根据历史运动轨迹,判断目标是否运动
  void UpdateMotionBehavior(MovingObject *moving_object_,double time_stamp_ );

private:
  int map_width;
  int map_height;
  int map_offset_y;
  float map_resolution;
  CvPoint  ego_veh_position;

  State_Vehicle ego_veh_state_current;
  double time_stamp;

  ShowResult show_result;
};
