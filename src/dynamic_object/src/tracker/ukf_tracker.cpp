/*
 * ukf_tracker.cpp
 *
 *  Created on: 2018年6月28日
 *      Author: zhanghm
 */

#include "ukf_tracker.h"
#include "measurement_package.h"
UKFTracker::UKFTracker() {
  // TODO Auto-generated constructor stub

}

UKFTracker::~UKFTracker() {
  // TODO Auto-generated destructor stub
}

void UKFTracker::ProcessTracking(vector<CandidateObject>* candidat_object_vector_,vector<MovingObject>* moving_object_vector_,double time_stamp){
  for (int i = 0; i < (moving_object_vector_->size()); i++){
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    MeasurementPackage meas_package;
    // read measurements at this timestamp
    meas_package.sensor_type_ = MeasurementPackage::LASER;
    meas_package.raw_measurements_ = VectorXd(2);
    meas_package.timestamp_ = time_stamp;
    float px,py;
    if(0 == temp_moving_object.track_state.tracked_times){//新添加目标
      px = temp_moving_object.center_point_ab.x;
      py = temp_moving_object.center_point_ab.y;
    }
    else{
      if(temp_moving_object.is_updated){//有匹配目标
        int match_index = temp_moving_object.candidate_index;//找到对应匹配目标索引
        CandidateObject temp_candidate = candidat_object_vector_->at(match_index);
        Correct_ContourPoint4( &temp_moving_object, &temp_candidate);//加入观测量,修改了center_point_ab
        px = temp_moving_object.center_point_ab.x;
        py = temp_moving_object.center_point_ab.y;
      }
      else{//目标检测丢失一次,使用上一时刻最优估计值作为观测量
        px = temp_moving_object.ukf_center.x_(0);
        py = temp_moving_object.ukf_center.x_(1);
      }
    }
    meas_package.raw_measurements_ << px, py;
    temp_moving_object.ukf_center.ProcessMeasurement(meas_package);//UKF滤波更新
  }
}

void UKFTracker::Correct_ContourPoint4(MovingObject *moving_object_, CandidateObject* candidate_object_){
  moving_object_->object_rect = candidate_object_->object_rect;
  moving_object_->object_box = candidate_object_->object_box;

  //边长补偿：暂时没用
  int point_index_pre2cd = moving_object_->point_index_pre2cd;
  moving_object_->is_same_diag_index = false;
  if(abs(point_index_pre2cd)==0 || abs(point_index_pre2cd)==2)
      moving_object_->is_same_diag_index = true;
  float line_length_cd[2];
  line_length_cd[0] = candidate_object_->shape.line_length0;
  line_length_cd[1] = candidate_object_->shape.line_length1;
  bool is_entire_line_cd[2];
  is_entire_line_cd[0] = candidate_object_->shape.is_entire_line0;
  is_entire_line_cd[1] = candidate_object_->shape.is_entire_line1;
  if (!moving_object_->is_same_diag_index)
  {
      line_length_cd[0] = candidate_object_->shape.line_length1;
      line_length_cd[1] = candidate_object_->shape.line_length0;

      is_entire_line_cd[0] = candidate_object_->shape.is_entire_line1;
      is_entire_line_cd[1] = candidate_object_->shape.is_entire_line0;
  }

  float line_length_mt[2];
  line_length_mt[0] = moving_object_->shape.line_length0;
  line_length_mt[1] = moving_object_->shape.line_length1;

  bool is_entire_line_mt[2];
  is_entire_line_mt[0] = moving_object_->shape.is_entire_line0;
  is_entire_line_mt[1] = moving_object_->shape.is_entire_line1;

  //更新当前检测参数：中心点、四个顶点
  for (int m = 0; m < 4; ++m)
  {
      int index_cd = BaseFunction::value_in_threshod_int(m + point_index_pre2cd, 0, 3);
      moving_object_->shape.point4[m] = candidate_object_->shape.point4[index_cd];
      moving_object_->shape.polar4[m] = candidate_object_->shape.polar4[index_cd];
      moving_object_->shape.fit_line4[m] = candidate_object_->shape.fit_line4[index_cd];
  }

  moving_object_->shape.line_length0 = line_length_mt[0];
  moving_object_->shape.line_length1 = line_length_mt[1];
  moving_object_->shape.is_entire_line0 = is_entire_line_mt[0];
  moving_object_->shape.is_entire_line1 = is_entire_line_mt[1];

  //update 中心点
  int track_index0_cd = candidate_object_->track_index0;
  moving_object_->track_index0 = BaseFunction::value_in_threshod_int(track_index0_cd- point_index_pre2cd, 0, 3);

  moving_object_->track_point.x = candidate_object_->center_point.x;
  moving_object_->track_point.y = candidate_object_->center_point.y;
  moving_object_->center_point = candidate_object_->center_point;
  moving_object_->dis_veh_xy = candidate_object_->dis_veh_xy;
  moving_object_->position_angle = candidate_object_->position_angle;

  // update pos head
  float temp_x = moving_object_->shape.point4[1].x - moving_object_->shape.point4[0].x;
  float temp_y = moving_object_->shape.point4[1].y - moving_object_->shape.point4[0].y;
  float temp_angle = atan2(temp_y, temp_x);
  moving_object_->pose_head = temp_angle;

  //转变为全局坐标
  Point_d temp_center_g;
  BaseFunction::point_local_to_global(moving_object_->center_point.x, moving_object_->center_point.y,
                                  ego_veh_state_current, &temp_center_g.x, &temp_center_g.y);
  moving_object_->center_point_ab = temp_center_g;//目标中心点全局坐标

  for ( int m=0; m<4; m++)
  {
      Point2D temp_point = moving_object_->shape.point4[m];
      Point_d temp_point_g;
      BaseFunction::point_local_to_global(temp_point.x, temp_point.y, ego_veh_state_current, &temp_point_g.x, &temp_point_g.y);
      moving_object_->contour_rect_ab.point4[m] = temp_point_g;
  }
}
