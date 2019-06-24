#pragma once
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "StructMovingTargetDefine.h"

class ShowResult
{
public:
  ShowResult();
  ~ShowResult();
  void Init(float ogm_resolution_, float ogm_offest_y_);

  void ShowEgoVehicle(IplImage* img_, int map_offest_y_, float ogm_resolution_, float egocar_with_, float egocar_height_);
  void ShowPolarOGM(IplImage* img_, Polar_Cell* polar_ogm_, float polarogm_angle_, float polarogm_radius_,
      float polarogm_angle_resolution_, float polarogm_radius_resolution_);
  void ShowObject_candidate(IplImage* img_, CandidateObject temp_object , int object_ID_, CvScalar object_color_, int thinkness_);
  void ShowObject_moving(IplImage* img_, MovingObject temp_object, const State_Vehicle& ego_veh_state_current);

  //显示运动目标轨迹
  void Show_moving_vector(cv::Mat& img, vector<MovingObject> moving_object_vector_, const State_Vehicle& ego_veh_state_current);

private:
  int map_offset_y;
  float map_resolution;
};
