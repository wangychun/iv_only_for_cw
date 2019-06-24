#include "TrajectoryDetection.h"
#include <iostream>
using namespace std;
Traj_detect::Traj_detect()
{
}
Traj_detect::~Traj_detect()
{
}

void Traj_detect::Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
      const int& virtual_line_num_, const float& virtual_line_resolution_)
{
  map_width = boost::math::round(ogm_width_ / ogm_resolution_) +1;
  map_height = boost::math::round(ogm_height_ / ogm_resolution_) +1;
  map_resolution = ogm_resolution_;
  map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);
}

void Traj_detect::MapInput(const int& frame_counter_,const double&  time_stamp_, const State_Vehicle& ego_veh_state_)
{
  time_stamp = time_stamp_;
  ego_veh_state_current = ego_veh_state_;
}

int Traj_detect::Motion_behavior( MovingObject *moving_object_, int history_num_, CvPoint2D32f* motion_traj_)
{
    int motion_behavior = 0;
    float delta_x = 0;
    float delta_y = 0;
    for (int i = 0; i < history_num_-1; ++i)
    {
        delta_x += motion_traj_[i].x - motion_traj_[i+1].x;
        delta_y += motion_traj_[i].y - motion_traj_[i+1].y;
    }
    float dis_move = sqrt(delta_x*delta_x + delta_y*delta_y);
    if(dis_move > 1.5)//移动距离大于1.5米，则认为是动态目标
    {
        motion_behavior = 1;
//        moving_object_->motion_track_times = moving_object_->track_state.tracked_times;
        float max_dis_rate = 0;
        for (int i = 0; i < history_num_-1; ++i)
        {
          float temp_x = motion_traj_[i].x - motion_traj_[i+1].x;
          float temp_y = motion_traj_[i].y - motion_traj_[i+1].y;
          float temp_dis = sqrt(temp_x*temp_x + temp_y*temp_y);
          float temp_rate = temp_dis/dis_move;
          max_dis_rate = max(temp_rate, max_dis_rate);
        }
        if(max_dis_rate>0.85)//TODO:0.45有待验证！计算相邻轨迹点距离梯度,即目标距离不会突然变化很大??zhanghm:对于人来说不适用
        {
//          moving_object_->motion_track_times = 0;
          motion_behavior = 0;
        }
    }
    return  motion_behavior;
}

void Traj_detect::Motion_traj_fit( MovingObject *moving_object_ , int history_num_, CvPoint2D32f* motion_traj_, float* traj_time_ )
{
    CvPoint2D32f* v_x_seed = (CvPoint2D32f*)malloc( history_num_ * sizeof(v_x_seed[0]));
    CvPoint2D32f* v_y_seed = (CvPoint2D32f*)malloc( history_num_ * sizeof(v_y_seed[0]));
    for (int i=0; i<history_num_; i++){
      v_x_seed[i].x = traj_time_[i]*10;
      v_y_seed[i].x = traj_time_[i]*10;
      v_x_seed[i].y = motion_traj_[i].x*2 ;
      v_y_seed[i].y = motion_traj_[i].y*2 ;
    }
    float fit_v_x_param[4];
    memset(fit_v_x_param, 0 , 4*sizeof(float));
    CvMat pointMat_x = cvMat( 1, history_num_, CV_32FC2, v_x_seed );
    cvFitLine( &pointMat_x, CV_DIST_L1, 1, 0.01, 0.01, fit_v_x_param );

    float fit_v_y_param[4];
    memset(fit_v_y_param, 0 , 4*sizeof(float));
    CvMat pointMat_y= cvMat( 1, history_num_, CV_32FC2, v_y_seed );
    cvFitLine( &pointMat_y, CV_DIST_L1, 1, 0.01, 0.01, fit_v_y_param );

    float temp_x_angle = atan2(fit_v_x_param[1], fit_v_x_param[0]);
    float temp_y_anlge = atan2(fit_v_y_param[1], fit_v_y_param[0]);
    float temp_v_x = 5*tan(temp_x_angle);
    float temp_v_y = 5*tan(temp_y_anlge);

//    moving_object_->fit_traj_time_x[3] = 0;
//    moving_object_->fit_traj_time_x[2] = 0;
//    moving_object_->fit_traj_time_x[1] = temp_v_x;
//    moving_object_->fit_traj_time_x[0] = -fit_v_x_param[2]*temp_v_y/10 + fit_v_x_param[3]/2;
//
//    moving_object_->fit_traj_time_y[3] = 0;
//    moving_object_->fit_traj_time_y[2] = 0;
//    moving_object_->fit_traj_time_y[1] = temp_v_y;
//    moving_object_->fit_traj_time_y[0] = -fit_v_y_param[2]*temp_v_y/10 + fit_v_y_param[3]/2;

    //轨迹预测
//    cout<<"predict_vvvvvvvvvvvvvvvvv "<<temp_v_x<<" "<<temp_v_y<<endl;
//    cout<<"kalmannnnnnnnnnnnnnnnnnnn "<<moving_object_->motion_state_ab.v_x_post<<" "<<moving_object_->motion_state_ab.v_y_post<<endl;
    float time_pred_resolution = 0.5;
//    for (int i = 0; i < moving_object_->predict_num; ++i){
//      moving_object_->predict_traj_fit[i].point.x = moving_object_->center_pt_meas_ab.x + time_pred_resolution*i*temp_v_x;
//      moving_object_->predict_traj_fit[i].point.y = moving_object_->center_pt_meas_ab.y + time_pred_resolution*i*temp_v_y;
//      moving_object_->predict_traj_fit[i].v_x = temp_v_x;
//      moving_object_->predict_traj_fit[i].v_y = temp_v_y;
//      moving_object_->predict_traj_fit[i].time_stamp = time_stamp + time_pred_resolution*i;
//
//      moving_object_->predict_traj[i].point.x = moving_object_->center_pt_meas_ab.x + time_pred_resolution*i*temp_v_x;
//      moving_object_->predict_traj[i].point.y = moving_object_->center_pt_meas_ab.y + time_pred_resolution*i*temp_v_y;
//      moving_object_->predict_traj[i].v_x = temp_v_x;
//      moving_object_->predict_traj[i].v_y = temp_v_y;
//      moving_object_->predict_traj[i].time_stamp = time_stamp + time_pred_resolution*i;
//      double delta_x = moving_object_->predict_traj[i].point.x - moving_object_->center_pt_meas_ab.x;
//      double delta_y = moving_object_->predict_traj[i].point.y - moving_object_->center_pt_meas_ab.y;
//      cout<<"delta_xxxxxxxxxxxxxxxx "<<delta_x<<" "<<delta_y<<endl;
//    }

    for(int i = 0;i<moving_object_->predict_info.size();++i){
      moving_object_->predict_info[i].time_stamp = time_stamp + time_pred_resolution*i;
      moving_object_->predict_info[i].point.x = moving_object_->center_pt_meas_ab.x + time_pred_resolution*i*temp_v_x;
      moving_object_->predict_info[i].point.y = moving_object_->center_pt_meas_ab.y + time_pred_resolution*i*temp_v_y;
      moving_object_->predict_info[i].v_x = temp_v_x;
      moving_object_->predict_info[i].v_y = temp_v_y;
      double delta_x = moving_object_->predict_info[i].point.x - moving_object_->center_pt_meas_ab.x;
      double delta_y = moving_object_->predict_info[i].point.y - moving_object_->center_pt_meas_ab.y;
//      cout<<"delta_xxxxxxxxxxxxxxxx2222222 "<<delta_x<<" "<<delta_y<<endl;
    }
}


void Traj_detect::UpdateMotionBehavior(MovingObject *moving_object_){
  time_stamp = moving_object_->timestamp;
  CvPoint2D32f motion_traj[30];
  float traj_time[30];
  memset(motion_traj, 0, 30*sizeof(CvPoint2D32f));
  memset(traj_time, 0, 30*sizeof(float));
  int object_type = abs(moving_object_->object_type);

  int history_num = min(20, (int)moving_object_->history_info.size());
  if(history_num <= 3){//小于三帧不处理
    return;
  }

  //历史轨迹集
  if (object_type < 4 ){//小目标取中心点作为运动轨迹作为分析点 TODO:from object_type < 4 change to 1
    //倒序,从最新开始
    for(auto iter = moving_object_->history_info.rbegin();iter!=moving_object_->history_info.rbegin()+history_num;++iter){
      int t = iter - moving_object_->history_info.rbegin();
      motion_traj[t].x = iter->history_center_ab.x - moving_object_->center_pt_meas_ab.x;
      motion_traj[t].y = iter->history_center_ab.y - moving_object_->center_pt_meas_ab.y;
      traj_time[t] = iter->time_stamp - time_stamp;
    }
  }

  if (object_type > 3)//大目标取最近点作为运动轨迹分析点
  {
    int point_index0 = moving_object_->track_index0;
    //倒序,从最新开始
    for(auto iter = moving_object_->history_info.rbegin();iter!=moving_object_->history_info.rbegin()+history_num;++iter){
      int t = iter - moving_object_->history_info.rbegin();
      motion_traj[t].x = iter->history_rect_ab.point4[point_index0].x -
                         moving_object_->contour_rect_ab.point4[point_index0].x;
      motion_traj[t].y = iter->history_rect_ab.point4[point_index0].y -
                         moving_object_->contour_rect_ab.point4[point_index0].y;
      traj_time[t] = iter->time_stamp - time_stamp;
    }
//    for (int i = 1; i < history_num; ++i)
//    {
//      motion_traj[i].x = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].x
//          - moving_object_->contour_rect_ab.point4[point_index0].x;
//      motion_traj[i].y = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].y
//          - moving_object_->contour_rect_ab.point4[point_index0].y;
//
//      traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
//    }
  }

  //运动判别
  //只判断静止目标是否是运动目标
  int motion_behavior = moving_object_->motion_behavior;
  if (moving_object_->motion_behavior<1){
    motion_behavior = Motion_behavior(moving_object_, history_num,  motion_traj);
  }
  moving_object_->motion_behavior = motion_behavior;
  //对判定为运动的目标进行预测速度拟合
  if(moving_object_->motion_behavior > 0){
    Motion_traj_fit(moving_object_, history_num,  motion_traj, traj_time);//预测轨迹
  }
}

void Traj_detect::UpdateMotionBehavior(MovingObject *moving_object_,double time_stamp_){
  time_stamp = time_stamp_;
  CvPoint2D32f motion_traj[30];
  float traj_time[30];
  memset(motion_traj, 0, 30*sizeof(CvPoint2D32f));
  memset(traj_time, 0, 30*sizeof(float));

  float delta_time = 0;
  int object_type = abs(moving_object_->object_type);
  int history_num = min(20, moving_object_->track_state.tracked_times);
  if(history_num <= 3){//小于三帧不处理
    return;
  }

  //历史轨迹集
  if (object_type < 4 )//小目标取中心点作为运动轨迹作为分析点
  {
    for (int i = 1; i < history_num; ++i)
    {
      motion_traj[i].x = moving_object_->history_state.history_center_ab[i-1].x - moving_object_->center_point_ab.x;
      motion_traj[i].y = moving_object_->history_state.history_center_ab[i-1].y - moving_object_->center_point_ab.y;
      traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
    }
  }
  if (object_type > 3)//大目标取最近点作为运动轨迹分析点
  {
    int point_index0 = moving_object_->track_index0;
    for (int i = 1; i < history_num; ++i)
    {
      motion_traj[i].x = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].x
          - moving_object_->contour_rect_ab.point4[point_index0].x;
      motion_traj[i].y = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].y
          - moving_object_->contour_rect_ab.point4[point_index0].y;

      traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
    }
  }

  //运动判别
  //只判断静止目标是否是运动目标
  int motion_behavior = moving_object_->motion_behavior;
  if (moving_object_->motion_behavior<1){
      motion_behavior = Motion_behavior(moving_object_, history_num,  motion_traj);
  }
  moving_object_->motion_behavior = motion_behavior;
  //对判定为运动的目标进行预测速度拟合
  if(moving_object_->motion_behavior > 0){
    Motion_traj_fit(moving_object_, history_num,  motion_traj, traj_time);//预测轨迹
  }
}

void Traj_detect::Motion_traj( MovingObject *moving_object_, State_Vehicle ego_veh_state_current_,double time_stamp_ )
{
  cout<<"enter in Motion_trajjjjjjjjjjjjjjjjjjjjjjjjjj"<<endl;
  ego_veh_state_current = ego_veh_state_current_;
    time_stamp = time_stamp_;
    int motion_behavior = moving_object_->motion_behavior;
    CvPoint2D32f motion_traj[30];
    float traj_time[30];
    memset(motion_traj, 0, 30*sizeof(CvPoint2D32f));
    memset(traj_time, 0, 30*sizeof(float));
    int history_num = 0;
    float delta_time = 0;
    int object_type = abs(moving_object_->object_type);

    cout<<"moving_object_status "<<moving_object_->is_updated<<" "<<moving_object_->motion_behavior<<endl;
    cout<<"moving_object_ "<<moving_object_->center_point_ab.x<<" "<<moving_object_->center_point_ab.y<<endl;
    cout<<"ego_veh_state_current_ "<<ego_veh_state_current_.global_position.dLng<<" "<<ego_veh_state_current_.global_position.dLat<<endl;
    //只判断静止目标是否是运动目标
    if (moving_object_->is_updated&& moving_object_->motion_behavior<1)//TODO:此处相当于目标为静止并且有更新或者是新加入目标才进行判别
    {
        if (moving_object_->track_state.tracked_times>3 )//跟踪次数三次以上
        {
            delta_time = time_stamp - moving_object_->history_state.history_time[5];
            history_num = min(20, moving_object_->track_state.tracked_times);
            if (object_type < 4 )//小目标取中心点作为运动轨迹作为分析点
            {
                for (int i = 1; i < history_num; ++i)
                {
                    motion_traj[i].x = moving_object_->history_state.history_center_ab[i-1].x - moving_object_->center_point_ab.x;
                    motion_traj[i].y = moving_object_->history_state.history_center_ab[i-1].y - moving_object_->center_point_ab.y;
                    traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
                }
            }
            if (object_type > 3)//大目标取最近点作为运动轨迹分析点
            {
                int point_index0 = moving_object_->track_index0;
                for (int i = 1; i < history_num; ++i)
                {
                    motion_traj[i].x = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].x
                                       - moving_object_->contour_rect_ab.point4[point_index0].x;

                    motion_traj[i].y = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].y
                                       - moving_object_->contour_rect_ab.point4[point_index0].y;

                    traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
                }
            }
            motion_behavior = Motion_behavior(moving_object_, history_num,  motion_traj);
        }//end if (moving_object_->track_state.tracked_times>3)
        if (history_num > 0)
             Motion_traj_fit(moving_object_, history_num,  motion_traj, traj_time);//预测轨迹
        moving_object_->motion_behavior = motion_behavior;
    } else//否则,维持运动状态不变,即只要判定为运动目标,则一直认为运动
    {
//        float delta_time = time_stamp_ - moving_object_->predict_traj[0].time_stamp;
//      float delta_time = 0.5;
//        for (int i = 0; i < moving_object_->predict_num; ++i)
//        {
//            moving_object_->predict_traj[i].point.x = moving_object_->predict_traj[0].point.x
//                                                      + i*delta_time*moving_object_->predict_traj[0].v_x;
//            moving_object_->predict_traj[i].point.y = moving_object_->predict_traj[0].point.y
//                                                      + i*delta_time*moving_object_->predict_traj[0].v_y;
//            moving_object_->predict_traj[i].time_stamp = moving_object_->predict_traj[i].time_stamp + delta_time;
//        }
    }
}

void Traj_detect::MotionTraj( MovingObject *moving_object_, State_Vehicle ego_veh_state_current_,double time_stamp_ )
{
  cout<<"enter in Motion_trajjjjjjjjjjjjjjjjjjjjjjjjjj"<<endl;
  ego_veh_state_current = ego_veh_state_current_;
  time_stamp = time_stamp_;
  CvPoint2D32f motion_traj[30];
  float traj_time[30];
  memset(motion_traj, 0, 30*sizeof(CvPoint2D32f));
  memset(traj_time, 0, 30*sizeof(float));
  int history_num = 0;
  float delta_time = 0;
  int object_type = abs(moving_object_->object_type);

  if(moving_object_->track_state.tracked_times>3){//跟踪次数大于3次进行动态分析
    history_num = min(20, moving_object_->track_state.tracked_times);//最多取历史20帧
    if (object_type < 4 ){//小目标取中心点作为运动轨迹作为分析点
      for (int i = 1; i < history_num; ++i){
        motion_traj[i].x = moving_object_->history_state.history_center_ab[i-1].x - moving_object_->center_point_ab.x;
        motion_traj[i].y = moving_object_->history_state.history_center_ab[i-1].y - moving_object_->center_point_ab.y;
        traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
      }
    }

    if (object_type > 3){//大目标取最近点作为运动轨迹分析点
      int point_index0 = moving_object_->track_index0;
      for (int i = 1; i < history_num; ++i){
        motion_traj[i].x = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].x
            - moving_object_->contour_rect_ab.point4[point_index0].x;
        motion_traj[i].y = moving_object_->history_state.history_rect_ab[i-1].point4[point_index0].y
            - moving_object_->contour_rect_ab.point4[point_index0].y;
        traj_time[i] = moving_object_->history_state.history_time[i-1] - time_stamp;
      }
    }
    int motion_behavior = Motion_behavior(moving_object_, history_num,  motion_traj);
    moving_object_->motion_behavior = motion_behavior;
  }

  if(moving_object_->motion_behavior > 0){//运动目标
    //对运动目标进行轨迹预测
    Motion_traj_fit(moving_object_, history_num,  motion_traj, traj_time);//预测轨迹
  }
}

