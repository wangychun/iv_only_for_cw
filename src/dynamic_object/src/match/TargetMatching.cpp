#include "TargetMatching.h"
#include <algorithm>
#include <visualization/visualizer_utils.h>
using namespace std;

TargetMatching::TargetMatching():
frame_counter(0),
is_show_result_(true),
time_stamp(0.0),
candidate_object_num(0),
moving_object_num(0),
map_width(0),
map_height(0),
map_offset_y(0),
map_resolution(0.0)
{
  target_ID_index = 0;
  target_ID = new TargetID[MAX_NUM_TARGET];
  for (int i = 0;i < MAX_NUM_TARGET;i++)
  {
    target_ID[i].ID_number = i;
    target_ID[i].locked = false;
  }

  init_matchstate_.is_match = false;
  init_matchstate_.match_index = -1;
  init_matchstate_.similar_num = 0;
  init_matchstate_.similar_index_vector.clear();
  init_matchstate_.max_similar_num = 0;
  init_matchstate_.max_similar_index_vector.clear();
}

TargetMatching::~TargetMatching()
{
  delete[] target_ID;
}

void TargetMatching::Init(float ogm_width_, float ogm_height_, float ogm_offest_y_,
    float ogm_resolution_,int virtual_line_num_, float virtual_line_resolution_,bool is_show_result)
{
  map_width = boost::math::round(ogm_width_ / ogm_resolution_) +1;//401
  map_height = boost::math::round(ogm_height_ / ogm_resolution_) +1;//501
  map_resolution = ogm_resolution_;//0.2
  map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);//250

  virtual_line_num = virtual_line_num_; //360
  virtual_line_resolution = virtual_line_resolution_;//
  this->is_show_result_ = is_show_result;

  tracker_global.Init(ogm_width_, ogm_height_, ogm_offest_y_, ogm_resolution_, virtual_line_num_,virtual_line_resolution_);
}

void TargetMatching::Init(int map_width, int map_height, int map_offest_y, float map_resolution,
            int virtual_line_num_, float virtual_line_resolution_,bool is_show_result)
{
  this->map_width = map_width;//401
  this->map_height = map_height;//501
  this->map_offset_y = map_offest_y;//250
  this->map_resolution = map_resolution;//0.2

  this->virtual_line_num = virtual_line_num_; //360
  this->virtual_line_resolution = virtual_line_resolution_;//
  this->is_show_result_ = is_show_result;

  tracker_global.Init(map_width, map_height, map_offest_y, map_resolution, virtual_line_num_,virtual_line_resolution_);
}

void TargetMatching::MapInput(int frame_counter_,double  time_stamp_, State_Vehicle ego_veh_state_,
                              const std::vector<Line_int>& virtual_line_in)
{
  this->frame_counter = frame_counter_;
  this->time_stamp = time_stamp_;
  this->ego_veh_state_current = ego_veh_state_;

  boundary_line = new Line_int[virtual_line_num];
  for (size_t i = 0; i < virtual_line_in.size(); ++i) {
    boundary_line->angle = virtual_line_in[i].angle;
    boundary_line->left_invalid = virtual_line_in[i].left_invalid;
    boundary_line->line_length = virtual_line_in[i].line_length;
    boundary_line->right_invalid = virtual_line_in[i].right_invalid;
    boundary_line->type = virtual_line_in[i].type;
    boundary_line->x1 = virtual_line_in[i].x1;
    boundary_line->x2 = virtual_line_in[i].x2;
    boundary_line->y1 = virtual_line_in[i].y1;
    boundary_line->y2 = virtual_line_in[i].y2;
  }
}

int TargetMatching::IDGenerator()
{
  int is_over = 0;
  int temp_ID_number = 0;
  while (true){
    if (target_ID_index == MAX_NUM_TARGET){
      target_ID_index = 0;
    }
    if (!target_ID[target_ID_index].locked){
      temp_ID_number = target_ID[target_ID_index].ID_number;
      target_ID[target_ID_index].locked = true;
      target_ID_index++;
      break;
    }else{
      ++target_ID_index;
    }
    is_over++;//遍历了一遍之后200个目标全被占用,则默认返回0
    if (is_over > MAX_NUM_TARGET-1)
      break;
  }
  return temp_ID_number;
}

int TargetMatching::Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_)
{
  if (moving_object_.object_type <2 || candidate_object_.object_type<2)
    return 0;

  CvPoint2D32f line4_cd[4]; // 01 12
  CvPoint2D32f line4_mt[4]; // 01 12 23 30
  if (method_ == 2)
  {
    for (int m=0; m<4; m++)
    {
      int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
      line4_cd[m].x = candidate_object_.shape.point4[m_up].x - candidate_object_.shape.point4[m].x;
      line4_cd[m].y = candidate_object_.shape.point4[m_up].y - candidate_object_.shape.point4[m].y;
      line4_mt[m].x = moving_object_.shape.point4[m_up].x - moving_object_.shape.point4[m].x;
      line4_mt[m].y = moving_object_.shape.point4[m_up].y - moving_object_.shape.point4[m].y;
    }
  }
  if (method_ == 1)
  {
    for (int m=0; m<4; m++)
    {
      int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
      line4_cd[m].x = candidate_object_.shape.point4[m_up].x - candidate_object_.shape.point4[m].x;
      line4_cd[m].y = candidate_object_.shape.point4[m_up].y - candidate_object_.shape.point4[m].y;
      line4_mt[m].x = moving_object_.contour_rect_pre.point4[m_up].x - moving_object_.contour_rect_pre.point4[m].x;
      line4_mt[m].y = moving_object_.contour_rect_pre.point4[m_up].y - moving_object_.contour_rect_pre.point4[m].y;
    }
  }

  int match_index0_cd = candidate_object_.track_index1;
  int match_index0_mt = moving_object_.track_index1;
  int line_index0_cd = match_index0_cd;
  float delta_angle_line0[4];
  float min_delta_angle0 = 361;
  for (int m=0; m<4; m++)
  {
    delta_angle_line0[m] = BaseFunction::Angle_line2(line4_cd[line_index0_cd], line4_mt[m]);
    if (delta_angle_line0[m] < min_delta_angle0)
    {
      min_delta_angle0 = delta_angle_line0[m];
      match_index0_mt = m;
    }
  }

  int point_index_pre2cd = match_index0_cd - match_index0_mt;
  return point_index_pre2cd;
}

void TargetMatching::MatchAnalsis( vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_,bool show_match)
{
  tracker_global.ProcessMatch(candidate_object_vector, moving_object_vector_,match_state_cd_, match_state_mt_,false);
  if (0)
  {
    //用射线法判断没有被匹配的跟踪目标,再进行匹配(进行补搜)
    //在没有被匹配上的跟踪目标，在其射线扩大范围周围搜索有无检测目标,如果该目标没有被匹配,则认为匹配上
    for (int j = 0; j <moving_object_vector_->size() ; ++j)
    {
      MovingObject temp_moving_object = moving_object_vector_->at(j);
      if (!match_state_mt_[j].is_match && temp_moving_object.track_state.missed_times_c<3
          && temp_moving_object.dis_veh_xy<40)
      {
        CvPoint2D32f temp_center = temp_moving_object.center_point_pre;//上一时刻目标中心点
        int index0 = temp_moving_object.shape.polar4[0].point_index;
        float temp_center_angle = BaseFunction::Angle_atan2(temp_center.x, temp_center.y);
        Point2D base_point = temp_moving_object.contour_rect_pre.point4[index0];
        float temp_dis = sqrt(base_point.x*base_point.x + base_point.y*base_point.y);
        float temp_sin = 2/temp_dis;
        float temp_value= 0.5;
        temp_sin = min(temp_sin, temp_value);
        float temp_angle = asin(temp_sin)*180/pi;//用2m半径的圆计算需要扩大的射线角度
        int delta_index = (int)(temp_angle/virtual_line_resolution);
        delta_index = min(delta_index, 30);
        delta_index = max(delta_index, 5);

        //从跟踪目标中心点往两边扩大射线范围
        int line_center = (int)(temp_center_angle/virtual_line_resolution);
        int line_start = BaseFunction::value_in_threshod_int(line_center-delta_index, 0, virtual_line_num-1);
        int min_delta_dis = 100;
        int min_dis_index = -1;
        for (int m = 0; m < delta_index*2; ++m)
        {
          int line_index = BaseFunction::value_in_threshod_int(line_start+m, 0, virtual_line_num-1);
          float delta_dis_veh = temp_dis - boundary_line[line_index].line_length*map_resolution;
          if (fabs(delta_dis_veh) < 2)//跟踪上的目标最近点和射线扫到的检测目标的径向距离范围
          {
            if (fabs(delta_dis_veh) < min_delta_dis)
            {
              min_delta_dis = abs(delta_dis_veh);
              min_dis_index = line_index;
            }

          }
        }

        if (min_dis_index >-1)//在扩大范围内搜索到当前检测的目标
        {
          CvPoint temp_point , temp_point1;
          temp_point1.x = boundary_line[line_center].x1;
          temp_point1.y = boundary_line[line_center].y1;
          temp_point.x = boundary_line[line_center].x2;
          temp_point.y = boundary_line[line_center].y2;
          //                    cvLine(img_result,temp_point, temp_point1, cvScalar(0,255,0), 1, 8,0);
          if (virtual_line_type[min_dis_index] > -1)
          {
            int index_cd = virtual_line_type[min_dis_index];
            CandidateObject temp_candidate = candidate_object_vector->at(index_cd);
            float delta_dis_veh = temp_candidate.dis_veh_xy - temp_dis;
            if (!match_state_cd_[index_cd].is_match && fabs(delta_dis_veh)<3)
            {
              match_state_mt_[j].is_match = true;
              match_state_mt_[j].match_index = index_cd;
              match_state_cd_[index_cd].is_match = true;
              match_state_cd_[index_cd].match_index = j;
            }
          }
        } else
        {
          if (virtual_line_type[line_center] > -1)
          {
            int index_cd = virtual_line_type[min_dis_index];
            CandidateObject temp_candidate = candidate_object_vector->at(index_cd);
            float delta_dis_veh = temp_candidate.dis_veh_xy - temp_moving_object.dis_veh_xy;
            if (!match_state_cd_[index_cd].is_match && fabs(delta_dis_veh)<5 &&
                abs(temp_candidate.object_type)>3)
            {
              match_state_mt_[j].is_match = true;
              match_state_mt_[j].match_index = index_cd;
              match_state_cd_[index_cd].is_match = true;
              match_state_cd_[index_cd].match_index = j;
            }
          }
        }
      }
    }//end for (int j = 0; j <moving_object_vector_->size() ; ++j)
  }
}

void TargetMatching::Init_MovingObject(MovingObject *moving_object_, CandidateObject* candidate_object_)
{
  moving_object_->target_ID = IDGenerator();//给该目标赋一个ID号,目标剔除的时候,会释放该ID号
  moving_object_->is_new_add = true;
  moving_object_->has_match = false;
  moving_object_->timestamp = time_stamp;

  moving_object_->is_updated = true;
  moving_object_->center_pt = candidate_object_->center_point;
  moving_object_->center_pt_meas_ab = candidate_object_->center_point_ab;
//  //目标全局坐标
//  Point_d temp_point_d;
//  BaseFunction::point_local_to_global(moving_object_->center_pt.x, moving_object_->center_pt.y, ego_veh_state_current,
//      &temp_point_d.x, &temp_point_d.y);
//  moving_object_->center_pt_meas_ab = temp_point_d;//作为观测量
  moving_object_->center_point = candidate_object_->center_point;
  moving_object_->center_point_ab = candidate_object_->center_point_ab;

  moving_object_->shape = candidate_object_->shape;
  moving_object_->shape.head_index = -1;
  //目标矩形框全局坐标
  Contour_Rect temp_contour_rect;
  for(int m = 0;m < 4; ++m){
    temp_contour_rect.point4[m] = candidate_object_->shape.point4[m];
    moving_object_->contour_rect_ab.point4[m] = candidate_object_->contour_rect_ab.point4[m];
  }

  moving_object_->ego_veh_start = ego_veh_state_current;
  moving_object_->center_start_ab = candidate_object_->center_point_ab;
  moving_object_->rect_start_ab = moving_object_->contour_rect_ab;
//
//  for ( int m=0; m<4; m++)
//  {
//    temp_contour_rect.point4[m] = moving_object_->shape.point4[m];
//    Point_d temp_point_g;
//    BaseFunction::point_local_to_global(temp_contour_rect.point4[m].x, temp_contour_rect.point4[m].y, ego_veh_state_current,
//        &temp_point_g.x, &temp_point_g.y);
//    moving_object_->contour_rect_ab.point4[m] = temp_point_g;
//  }
//  moving_object_->contour_rect_ab = candidate_object_->contour_rect_ab;


  moving_object_->is_KF_center_ab_initialized = false;
  moving_object_->is_KF_pt4_ab_initialized = false;
  //预测PREDICT_NUM帧信息
  moving_object_->predict_info.resize(PREDICT_NUM);
//  moving_object_->predict_num = PREDICT_NUM;
//  memset( moving_object_->predict_traj, 0 , PREDICT_NUM*sizeof(Predict_traj) );
  moving_object_->object_rect = candidate_object_->object_rect;
  moving_object_->object_box = candidate_object_->object_box;


  moving_object_->point_index_pre2cd = 0;
  moving_object_->is_same_diag_index = true;
//  moving_object_->is_entire_total = false;


  moving_object_->track_point = candidate_object_->track_point;
  moving_object_->pose_head = 0;
  moving_object_->track_index0 = candidate_object_->track_index0;
  moving_object_->track_index1 = candidate_object_->track_index1;


  moving_object_->object_type = candidate_object_->object_type;
//  moving_object_->occluded_state = candidate_object_->occluded_state;
  moving_object_->dangerous_level = 0;
  moving_object_->motion_behavior = 0;
  moving_object_->send_times = 0;

  moving_object_->dis_veh_xy = candidate_object_->dis_veh_xy;
  moving_object_->position_angle = candidate_object_->position_angle;
  moving_object_->region_index = candidate_object_->region_index;

//  //Kalman滤波状态
//  moving_object_->filter_length2[0]= -1;
//  moving_object_->filter_length2[1]= -1;

  for(int i = 0;i<4;++i) moving_object_->filter_state_pt4_ab[i]=-1;
//  memset(&moving_object_->motion_state, 0, sizeof(Motion_State));
//  moving_object_->motion_state.x_post = moving_object_->track_point.x;
//  moving_object_->motion_state.y_post = moving_object_->track_point.y;
//  moving_object_->motion_state.pose_head_post = moving_object_->pose_head;
//  moving_object_->motion_state.x_pred = moving_object_->motion_state.x_post;
//  moving_object_->motion_state.y_pred = moving_object_->motion_state.y_post;

  moving_object_->filter_pos_head_ab = -1;
  memset(&moving_object_->motion_state_ab, 0, sizeof(Motion_State));

  /***************
   * 跟踪状态初始化
   ***************/
  moving_object_->time_stamp = time_stamp;
  moving_object_->time_stamp_start = time_stamp;
  moving_object_->ID_number = 0;
  moving_object_->track_state.tracked_times = 0;
  moving_object_->track_state.tracked_times_c = 0;
  moving_object_->track_state.missed_times = 0;
  moving_object_->track_state.missed_times_c = 0;
  moving_object_->track_state.tracked_state = 0;
  moving_object_->track_state.confidence_level = 0;



//  Point_d temp_center_ab;
//
//  BaseFunction::point_local_to_global(moving_object_->center_point.x, moving_object_->center_point.y, ego_veh_state_current,
//      &temp_center_ab.x, &temp_center_ab.y);
//
//  moving_object_->center_point_ab = temp_center_ab;

  moving_object_->history_state.history_num = 1;
  //历史信息初始化
  for (int history_id=0; history_id<HISTORY_NUM; history_id++)
  {
    moving_object_->history_state.history_time[history_id] = time_stamp;
    moving_object_->history_state.history_head[history_id] = moving_object_->motion_state.pose_head_post;
    moving_object_->history_state.history_v[history_id] = 0;
    moving_object_->history_state.history_rect[history_id] = temp_contour_rect;
    moving_object_->history_state.history_center[history_id] = moving_object_->center_point;
    moving_object_->history_state.history_center_ab[history_id] = moving_object_->center_point_ab;
    moving_object_->history_state.history_rect_ab[history_id] = moving_object_->contour_rect_ab;
    moving_object_->history_state.history_ego_state[history_id] = ego_veh_state_current;
  }


  moving_object_->dis_move = 0;
  for ( int m=0; m<4; m++)
  {
    moving_object_->match_range.range_dis[m] = 2;
    moving_object_->match_range.range_rect4[m].x = 2;
    moving_object_->match_range.range_rect4[m].y = 2;
  }
  moving_object_->match_range.d_radius = 5;
}


void TargetMatching::UpdateTrackStatus(MovingObject *moving_object_)
{
//  if (moving_object_->track_state.tracked_times_c>5)
//  {
//    moving_object_->track_state.missed_times = 0;
//  }
//
//  if (!moving_object_->has_match)//没有匹配目标
//  {
//    int line_index = (int)(moving_object_->position_angle / virtual_line_resolution);
//    float temp_dis_xy = moving_object_->shape.polar4[0].dis_xy;
//    if (line_index <virtual_line_num)
//    {
//      if (temp_dis_xy > (boundary_line[line_index].line_length*map_resolution +1))
//      {
//        moving_object_->track_state.missed_times = moving_object_->track_state.missed_times + 10;
//      }
//    }
//  }


  //目标跟踪状态0-5,0为临界状态,1-3跟踪越来越稳，一直稳定跟踪则维持在状态3,5则需要剔除该目标
  //1)初始状态为0,可能升级为1或者直接需要剔除
  switch(moving_object_->track_state.tracked_state){
    case 0:{
      if (moving_object_->track_state.tracked_times_c >= 5){//连续跟踪x次数以上,升级为1
        moving_object_->track_state.tracked_state = 1;
        moving_object_->track_state.tracked_times_c = 0;
        moving_object_->track_state.missed_times_c = 0;
      }
      if ((moving_object_->track_state.missed_times_c /*- moving_object_->track_state.tracked_times*/)>= 2){
        moving_object_->track_state.tracked_state = 5;
      }
      break;
    }
    case 1:{
      if ((moving_object_->track_state.tracked_times_c>=5)){
        moving_object_->track_state.tracked_state = 2;
        moving_object_->track_state.missed_times_c = 0;
        moving_object_->track_state.tracked_times_c = 0;
      }
      if((moving_object_->track_state.missed_times_c > 4)){
        moving_object_->track_state.tracked_state = 0;//降为0
        moving_object_->track_state.missed_times_c = 0;//重新计数
      }
//      if ((moving_object_->track_state.missed_times_c - moving_object_->track_state.tracked_times)>3
//          && moving_object_->track_state.missed_times>20)
//      {
//        moving_object_->track_state.tracked_state = 5;
//      }
      break;
    }
    case 2:{
      if (moving_object_->track_state.tracked_times>35 || moving_object_->track_state.tracked_times_c>10)
      {
        moving_object_->track_state.tracked_state = 3;
        moving_object_->track_state.missed_times = 0;//将丢失次数清0
        moving_object_->track_state.missed_times_c = 0;
      }
      if(moving_object_->track_state.missed_times_c>=7)
      {
        moving_object_->track_state.tracked_state = 1;
      }
      break;
    }
    case 3:{//稳定跟踪状态,除非满足一定条件变成4不稳定状态,否则保持该状态
      if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>10)
      {
        moving_object_->track_state.tracked_state = 4;
        moving_object_->track_state.tracked_times_c = 0;
        moving_object_->track_state.missed_times_c = 0;
      }
      break;
    }
    case 4:{//不稳定跟踪目标,可能是被遮挡,超出范围,检测不稳等原因
      if (moving_object_->track_state.tracked_times_c > 5 ){
        moving_object_->track_state.tracked_state = 3;
      }
      if (moving_object_->track_state.missed_times_c>=4){
        moving_object_->track_state.tracked_state = 5;
      }
      break;
    }
  }

//  if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>20)
//  {
//    moving_object_->track_state.tracked_state = 5;
//  }
}

void TargetMatching::TrakedState( MovingObject *moving_object_ )
{
  if (moving_object_->track_state.tracked_times_c>5)
  {
    moving_object_->track_state.missed_times = 0;
  }

  if (!moving_object_->is_updated)//没有匹配目标
  {
    int line_index = (int)(moving_object_->position_angle / virtual_line_resolution);
    float temp_dis_xy = moving_object_->shape.polar4[0].dis_xy;
    if (line_index <virtual_line_num)
    {
      if (temp_dis_xy > (boundary_line[line_index].line_length*map_resolution +1))
      {
        moving_object_->track_state.missed_times = moving_object_->track_state.missed_times + 10;
      }
    }
  }

#if 1
  if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>20)
  {
    moving_object_->track_state.tracked_state = 5;
  }
  if (moving_object_->track_state.tracked_state == 4)
  {
    if (moving_object_->track_state.tracked_times_c > 5 )
    {
      moving_object_->track_state.tracked_state = 3;
    }
    if (moving_object_->track_state.missed_times>25 || moving_object_->track_state.missed_times_c>15)
    {
      moving_object_->track_state.tracked_state = 5;
    }
  }
  if (moving_object_->track_state.tracked_state == 3)
  {
    if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>10)
    {
      moving_object_->track_state.tracked_state = 4;
    }
    if (moving_object_->track_state.tracked_times_c > 5 )
    {
      moving_object_->track_state.tracked_state = 2;
    }
  }
  if (moving_object_->track_state.tracked_state == 2)
  {
    if (moving_object_->track_state.missed_times>15 || moving_object_->track_state.missed_times_c>5)
    {
      moving_object_->track_state.tracked_state = 3;
    }
  }
  if (moving_object_->track_state.tracked_state == 1)
  {
    if ((moving_object_->track_state.tracked_times>5 || moving_object_->track_state.tracked_times_c>2)
        && moving_object_->track_state.missed_times_c<10)
    {
      moving_object_->track_state.tracked_state = 2;
    }
    if ((moving_object_->track_state.missed_times_c - moving_object_->track_state.tracked_times)>3
        && moving_object_->track_state.missed_times>20)
    {
      moving_object_->track_state.tracked_state = 5;
    }
  }
  if (moving_object_->track_state.tracked_state == 0)//默认为0,新添加目标
  {
    if (moving_object_->track_state.tracked_times > 3)//跟踪了3次以上,升级为1
    {
      moving_object_->track_state.tracked_state = 1;
    }
    if ((moving_object_->track_state.missed_times_c - moving_object_->track_state.tracked_times)> 3)
    {
      moving_object_->track_state.tracked_state = 5;
    }
  }
#endif

#if 0
  //目标跟踪状态0-5,0为临界状态,1-3跟踪越来越稳，一直稳定跟踪则维持在状态3,5则需要剔除该目标
  //1)初始状态为0,可能升级为1或者直接需要剔除
  switch(moving_object_->track_state.tracked_state){
    case 0:{
      if (moving_object_->track_state.tracked_times > 3)//跟踪了3次以上,升级为1
        moving_object_->track_state.tracked_state = 1;
      if ((moving_object_->track_state.missed_times_c - moving_object_->track_state.tracked_times)> 3){
        moving_object_->track_state.tracked_state = 5;
      }
      break;
    }
    case 1:{
      if ((moving_object_->track_state.tracked_times>5 || moving_object_->track_state.tracked_times_c>2)
          && moving_object_->track_state.missed_times_c<10)
      {
        moving_object_->track_state.tracked_state = 2;
      }
      if ((moving_object_->track_state.missed_times_c - moving_object_->track_state.tracked_times)>3
          && moving_object_->track_state.missed_times>20)
      {
        moving_object_->track_state.tracked_state = 5;
      }
      break;
    }
    case 2:{
      if (moving_object_->track_state.missed_times>15 || moving_object_->track_state.missed_times_c>5)
      {
        moving_object_->track_state.tracked_state = 3;
      }
      break;
    }
    case 3:{
      if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>10)
      {
        moving_object_->track_state.tracked_state = 4;
      }
      if (moving_object_->track_state.tracked_times_c > 5 )
      {
        moving_object_->track_state.tracked_state = 2;
      }
      break;
    }
    case 4:{
      if (moving_object_->track_state.tracked_times_c > 5 )
      {
        moving_object_->track_state.tracked_state = 3;
      }
      if (moving_object_->track_state.missed_times>25 || moving_object_->track_state.missed_times_c>15)
      {
        moving_object_->track_state.tracked_state = 5;
      }
      break;
    }
  }

  if (moving_object_->track_state.missed_times>20 || moving_object_->track_state.missed_times_c>20)
  {
    moving_object_->track_state.tracked_state = 5;
  }
#endif
}


void TargetMatching::Correct_ContourPoint4( MovingObject *moving_object_, CandidateObject* candidate_object_){
  //更新中心点
  moving_object_->center_pt = candidate_object_->center_point;
  moving_object_->center_point = candidate_object_->center_point;
  //中心点转变为全局坐标
//  Point_d temp_center_g;
//  BaseFunction::point_local_to_global(moving_object_->center_point.x, moving_object_->center_point.y,
//      ego_veh_state_current, &temp_center_g.x, &temp_center_g.y);
  moving_object_->center_point_ab = candidate_object_->center_point_ab;//目标中心点全局坐标
  moving_object_->center_pt_meas_ab = candidate_object_->center_point_ab;


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

  //更新矩形框四个顶点
  for (int m = 0; m < 4; ++m){
    int index_cd = BaseFunction::value_in_threshod_int(m + point_index_pre2cd, 0, 3);
    moving_object_->shape.point4[m] = candidate_object_->shape.point4[index_cd];
    moving_object_->shape.polar4[m] = candidate_object_->shape.polar4[index_cd];
    moving_object_->shape.fit_line4[m] = candidate_object_->shape.fit_line4[index_cd];

    moving_object_->contour_rect_ab.point4[m] = candidate_object_->contour_rect_ab.point4[index_cd];
  }
//  //矩形框转变为全局坐标
//  for ( int m=0; m<4; m++)
//  {
//    Point2D temp_point = moving_object_->shape.point4[m];
//    Point_d temp_point_g;
//    BaseFunction::point_local_to_global(temp_point.x, temp_point.y, ego_veh_state_current, &temp_point_g.x, &temp_point_g.y);
//    moving_object_->contour_rect_ab.point4[m] = temp_point_g;
//  }

  moving_object_->shape.line_length0 = line_length_mt[0];
  moving_object_->shape.line_length1 = line_length_mt[1];
  moving_object_->shape.is_entire_line0 = is_entire_line_mt[0];
  moving_object_->shape.is_entire_line1 = is_entire_line_mt[1];

  moving_object_->track_point.x = candidate_object_->center_point.x;
  moving_object_->track_point.y = candidate_object_->center_point.y;

  moving_object_->dis_veh_xy = candidate_object_->dis_veh_xy;
  moving_object_->position_angle = candidate_object_->position_angle;

  // update pos head
  float temp_x = moving_object_->shape.point4[1].x - moving_object_->shape.point4[0].x;
  float temp_y = moving_object_->shape.point4[1].y - moving_object_->shape.point4[0].y;
  float temp_angle = atan2(temp_y, temp_x);
  moving_object_->pose_head = temp_angle;
}

void TargetMatching::UpdateMeasurement(MovingObject& moving_object,const vector<CandidateObject>& candidate_object){
  int match_index = moving_object.candidate_index;//找到对应匹配目标索引
  CandidateObject temp_candidate = candidate_object.at(match_index);
  Correct_ContourPoint4( &moving_object, &temp_candidate);//加入观测量
}

void TargetMatching::UpdateMatchState( vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_)
{
  /**********************************************
   * 根据匹配分析的结果对已跟踪目标进行跟踪次数等状态更新
   **********************************************/
  for (int j = 0; j < moving_object_vector_->size(); j++){
    MovingObject temp_moving_object = moving_object_vector_->at(j);
    temp_moving_object.timestamp = time_stamp;//目标时间戳更新
    temp_moving_object.is_new_add = false;//不是新目标了

    temp_moving_object.time_stamp = time_stamp;
    if (match_state_mt_.at(j).is_match){//该运动目标在当前候选目标中有匹配
      temp_moving_object.has_match = true;

      int index_cd = match_state_mt_.at(j).match_index;
      temp_moving_object.candidate_index = index_cd;
      CandidateObject temp_candidate = candidate_object_vector->at(index_cd);
      candidate_object_vector->at(index_cd).match_index = j;//给该检测目标匹配上哪一个跟踪上的目标
      int point_index_pre2cd = Match_ContourPoint4( temp_moving_object, temp_candidate, 1);
      temp_moving_object.point_index_pre2cd  = point_index_pre2cd;

      temp_moving_object.is_updated = true;
      temp_moving_object.track_state.tracked_times_c++;
      temp_moving_object.track_state.missed_times_c = 0;
      temp_moving_object.track_state.tracked_times++;
    }
    else{//上一帧已经跟踪的目标,该时刻丢失
      temp_moving_object.has_match = false;

      temp_moving_object.candidate_index = -1;//没有更新
      temp_moving_object.is_updated = false;
      temp_moving_object.track_state.tracked_times_c = 0;
      temp_moving_object.track_state.missed_times_c++;
      temp_moving_object.track_state.missed_times ++;
    }
    moving_object_vector_->at(j) = temp_moving_object;

    //将已经跟踪上的目标（即上一帧的结果）投到当前栅格地图上用来可视化
    if (0){
      CvPoint temp_point4[4];
      for (int m = 0; m < 4; ++m)
      {
        CvPoint2D32f temp_point_f = cvPoint2D32f(temp_moving_object.contour_rect_pre.point4[m].x,
            temp_moving_object.contour_rect_pre.point4[m].y);
        CvPoint temp_point;
        BaseFunction::point_veh_to_img(img_result, &temp_point, temp_point_f, map_resolution, map_offset_y);
        temp_point4[m] = temp_point;
      }
      CvPoint* ppt_test = temp_point4;

      int point_count = 4;
      if (temp_moving_object.is_updated)//跟踪目标有匹配检测目标
      {
        cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(255,0,255), 1);//洋红色
      } else//目标出现了丢失
      {
        if (temp_moving_object.track_state.missed_times_c<3)
          cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,0,255), 1);//红色
        else
          cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,255,255), 1);//黄色
      }
    }
  }//end for (int j = 0; j < moving_object_vector_->size(); j++)


  if(this->is_show_result_){//显示匹配图
    this->ShowMatchMap(candidate_object_vector,moving_object_vector_);
  }

  /****************************************
   * 对当前跟踪上的目标根据一些条件进行必要的剔除
   ****************************************/
  for (int i=0; i<moving_object_vector_->size(); i++)
  {
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    //根据目标跟踪和丢失次数(后续可以改成利用概率),更新目标跟踪状态
//    TrakedState( &temp_moving_object );
    UpdateTrackStatus(&temp_moving_object);
    bool  is_erase = false;
    //满足需要栅除目标的条件
    if (temp_moving_object.track_state.tracked_state == 5)
      is_erase = true;

    if (!is_erase){
      moving_object_vector_->at(i) = temp_moving_object;
    }
    else{//删除目标,释放对它们的Kalman滤波跟踪器
      if ( temp_moving_object.filter_pos_head_ab> 0 )
        cvReleaseKalman(&temp_moving_object.kalman_pos_head_ab);

      for ( int m=0; m<4; m++)
      {
        if ( temp_moving_object.filter_state_pt4_ab[m]> 0 )
          cvReleaseKalman(&temp_moving_object.kalman_pt4_ab[m]);
      }

      //释放锁定的ID号
      assert(temp_moving_object.target_ID>=0);
      target_ID[temp_moving_object.target_ID].locked = false;
      moving_object_vector_->erase(moving_object_vector_->begin() + i);
      i--;
    }
  }

  /****************************************
   * 利用目标匹配状态,进行运动目标观测值更新
   ****************************************/
  for(int i = 0;i< moving_object_vector_->size();++i){
    MovingObject temp_obj = moving_object_vector_->at(i);
    if(temp_obj.has_match){//有匹配目标,利用匹配目标进行状态更新
      UpdateMeasurement(temp_obj,*candidate_object_vector);
    }
    moving_object_vector_->at(i) = temp_obj;
  }
  /************
   * 新目标添加
   ************/
  for (int i = 0; i < candidate_object_vector->size(); i++)//遍历当前检测到的目标
  {
    CandidateObject temp_candidate = candidate_object_vector->at(i);
    if (!match_state_cd_[i].is_match && temp_candidate.object_type>0)
    {
      MovingObject temp_moving_object;
      temp_moving_object.candidate_index = i;

      Init_MovingObject(&temp_moving_object, &temp_candidate);
      moving_object_vector_->push_back(temp_moving_object);//新检测到目标
    }
  }
  cout<<"after UpdateMatchState ..."<<endl;
}

void TargetMatching::InitMatchParam(int moving_obj_num,int candidate_obj_num){
  match_state_mt_ = std::vector<MatchState>(moving_obj_num,init_matchstate_);
  match_state_cd_ = std::vector<MatchState>(candidate_obj_num,init_matchstate_);
}

void TargetMatching::Release()
{
  delete[] boundary_line;
  delete[] virtual_line_type;

  cvReleaseImage(&img_result);
}


void TargetMatching::MatchProcess( IplImage *img_ogm_,
                                   vector<CandidateObject>* candidate_object_vector,
                                   vector<MovingObject>* moving_object_vector_)
{
  //Allocate memories
  img_result = cvCreateImage(cvGetSize(img_ogm_), 8, 3);
  cvCvtColor(img_ogm_, img_result, CV_GRAY2BGR);
  //存储原始栅格地图
  img_ogm_origin_ = cv::cvarrToMat(img_ogm_,true);
  cvtColor(img_ogm_origin_,img_result_show_,CV_GRAY2BGR);//转成三通道图

//  boundary_line = new Line_int[virtual_line_num];
//  memcpy( boundary_line, boundary_line_, virtual_line_num*sizeof(Line_int) );
  //初始化射线类型都为-1
  virtual_line_type = new int[virtual_line_num];
  memset(virtual_line_type, -1, virtual_line_num*sizeof(int));

  candidate_object_num = candidate_object_vector->size();
  moving_object_num = moving_object_vector_->size();

  if (1)//根据检测目标分布,给扇形区域赋值
  {
    for (int i = 0; i < candidate_object_vector->size(); ++i)//遍历候选目标框
    {
      CandidateObject temp_object = candidate_object_vector->at(i);
      //找到目标边界占据的扇形区域
      int line_start = (int)(temp_object.shape.polar4[1].angle/virtual_line_resolution);
      int line_end = (int)(temp_object.shape.polar4[2].angle/virtual_line_resolution);
      //得到目标占据多少个扇形区域
      int line_num = BaseFunction::value_in_threshod_int(line_end-line_start, 0, virtual_line_num-1) + 1;
      for (int k = 0; k < line_num; ++k)
      {
        int line_index = BaseFunction::value_in_threshod_int(line_start+k, 0, virtual_line_num-1);
        if (virtual_line_type[line_index] < 0)
        {
          virtual_line_type[line_index] = i;//将射线类型归给某个目标
        } else
        {
          int index_cd = virtual_line_type[line_index];
          CandidateObject temp_object1 = candidate_object_vector->at(index_cd);
          if (temp_object1.dis_veh_xy > temp_object.dis_veh_xy)
          {
            virtual_line_type[line_index]= i;//如果射线穿过两个物体，选择较近的
          }
        }
      }
    }
  }


  /***********************************************
   * 全局坐标转换:得到候选目标中心点和矩形框顶点的全局坐标
   ***********************************************/
  for (int i=0; i<candidate_object_vector->size(); i++){
    CandidateObject temp_candidate = candidate_object_vector->at(i);
    Point_d temp_center_g;
    //车体坐标系下目标中心点转全局坐标系
    BaseFunction::point_local_to_global(temp_candidate.center_point.x, temp_candidate.center_point.y,
                                        ego_veh_state_current, &temp_center_g.x, &temp_center_g.y);
    temp_candidate.center_point_ab = temp_center_g;
    //车体坐标系下目标框四个角点转全局坐标系
    for ( int m=0; m<4; m++){
      Point2D temp_point = temp_candidate.shape.point4[m];
      Point_d temp_point_g;
      BaseFunction::point_local_to_global(temp_point.x, temp_point.y, ego_veh_state_current, &temp_point_g.x, &temp_point_g.y);
      temp_candidate.contour_rect_ab.point4[m] = temp_point_g;
    }
    candidate_object_vector->at(i) = temp_candidate;
  }

  /******************************************************************
   *将已经跟踪的moving_object_vector中的全局目标,全都转到当前帧车体坐标系下,作为pre,用于与当前检测进行匹配分析
   *******************************************************************/
  for (int i=0; i<moving_object_vector_->size(); i++)//遍历每个认为是运动的目标
  {
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    //转换矩形框四个点
    for ( int m=0; m<4; m++){
      Point_d temp_point_g = temp_moving_object.contour_rect_ab.point4[m];
      Point2D temp_point;
      BaseFunction::point_global_to_local(temp_point_g.x, temp_point_g.y, ego_veh_state_current, &temp_point.x, &temp_point.y);
      temp_moving_object.contour_rect_pre.point4[m] = temp_point;
    }
    //转换中心点
    BaseFunction::point_global_to_local(temp_moving_object.center_pt_meas_ab.x, temp_moving_object.center_pt_meas_ab.y,
        ego_veh_state_current, &temp_moving_object.center_point_pre.x, &temp_moving_object.center_point_pre.y);

    //将上一时刻划定的匹配区域转到当前车体坐标系下,用上一时刻跟踪的目标在当前检测目标中进行匹配
    for (int m = 0; m < 4; ++m)
    {
      int m_pre = BaseFunction::value_in_threshod_int(m-1 , 0, 3);
      float temp_x = temp_moving_object.contour_rect_pre.point4[m].x - temp_moving_object.contour_rect_pre.point4[m_pre].x;
      float temp_y = temp_moving_object.contour_rect_pre.point4[m].y - temp_moving_object.contour_rect_pre.point4[m_pre].y;
      float temp_angle_pre = atan2(temp_y, temp_x);
      CvPoint2D32f line_point;
      //根据目标矩形框朝向,得到一个类似菱形的四边形匹配区域
      line_point.x = temp_moving_object.center_point_pre.x + temp_moving_object.match_range.range_dis[m]*cos(temp_angle_pre);
      line_point.y = temp_moving_object.center_point_pre.y + temp_moving_object.match_range.range_dis[m]*sin(temp_angle_pre);
      temp_moving_object.match_range.range_rect4[m] = line_point;
    }

    moving_object_vector_->at(i) = temp_moving_object;

    /**********************
     *     可视化
     ***********************/
    if (1)// 可视化匹配区域
    {
      CvPoint temp_point4[4];
      for (int m = 0; m < 4; ++m)
      {
        CvPoint temp_point;
        BaseFunction::point_veh_to_img(img_result, &temp_point, temp_moving_object.match_range.range_rect4[m],
                                       map_resolution, map_offset_y);
        temp_point4[m] = temp_point;
      }
      CvPoint* ppt_test = temp_point4;

      int point_count = 4;
      cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,255,255), 1);
    }

    //TODO:不应该放在这个地方,此时是还没有更新过的状态。将已经跟踪上的目标（即上一帧的结果）投到当前栅格地图上用来可视化
    if (0)
    {
      CvPoint temp_point4[4];
      for (int m = 0; m < 4; ++m)
      {
        CvPoint2D32f temp_point_f = cvPoint2D32f(temp_moving_object.contour_rect_pre.point4[m].x,
            temp_moving_object.contour_rect_pre.point4[m].y);
        CvPoint temp_point;
        BaseFunction::point_veh_to_img(img_result, &temp_point, temp_point_f, map_resolution, map_offset_y);
        temp_point4[m] = temp_point;
      }
      CvPoint* ppt_test = temp_point4;

      int point_count = 4;
      if (temp_moving_object.is_updated)//目标处于稳定跟踪状态
      {
        cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(255,0,255), 1);//洋红色
      } else//目标出现了丢失
      {
        if (temp_moving_object.track_state.missed_times_c<3)
          cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,0,255), 1);//红色
        else
          cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,255,255), 1);//黄色
      }
    }
  }//end for (int i=0; i<moving_object_vector_->size(); i++)


  InitMatchParam(moving_object_vector_->size(),candidate_object_vector->size());
  /*****************************************************
   * 匹配分析和匹配状态更新,在该步填充moving_object_vector_向量
   *****************************************************/
  if (candidate_object_vector->size() >0 && moving_object_vector_->size()>0) {
    this->MatchAnalsis( candidate_object_vector, moving_object_vector_,true);
  }
  //对进行匹配分析之后的候选目标和运动跟踪目标进行更新,决定是否要删除一些已跟踪目标或加入一些新检测目标
  UpdateMatchState( candidate_object_vector, moving_object_vector_);


  if (this->is_show_result_)//可视化
  {
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_PLAIN,1.8f,0.8f,0,2,8);

    char file_name_ID[50];
    sprintf(file_name_ID, "frame: %d", frame_counter);
    CvPoint word_point = cvPoint(5, 35);
    cvPutText(img_result, file_name_ID, word_point, &font, cvScalar(255,255,0) );

    sprintf(file_name_ID, "target: %d %d", moving_object_num, candidate_object_num);
    word_point = cvPoint(5, 65);
    cvPutText(img_result, file_name_ID, word_point, &font, cvScalar(255,255,0) );
    //绘制车辆
    show_result.ShowEgoVehicle(img_result, map_offset_y, map_resolution, EGO_CAR_WIDTH, EGO_CAR_LENGTH);
    cvNamedWindow("img_match_moving",CV_WINDOW_NORMAL);
    cvShowImage("img_match_moving", img_result);
  }
  Release();
}

void TargetMatching::ShowMatchMap(vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_)
{
  /**********************************************************************
   * 可视化匹配分析结果,还未进行目标剔除前,完全是匹配分析的结果,用来验证匹配分析的正确性
   **************************************************************************/
  cv::Mat img_test,img_compare;
  cv::cvtColor(img_ogm_origin_,img_test,CV_GRAY2BGR);
  cv::cvtColor(img_ogm_origin_,img_compare,CV_GRAY2BGR);

  char file_name_ID[50];
  sprintf(file_name_ID, "frame: %d", frame_counter);//显示帧数
  CvPoint word_point = cvPoint(5, 25);
  cv::putText(img_test,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  sprintf(file_name_ID, "target: %d %d", moving_object_num, candidate_object_num);//显示跟踪目标和当前检测目标数
  word_point = cvPoint(5, 45);
  cv::putText(img_test,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  /***************************
   * 绘制跟踪目标及显示跟踪目标信息
   ***************************/
//  CvFont font,font2;
//  cvInitFont(&font2, CV_FONT_HERSHEY_PLAIN, 0.5f, 1.5f, 0, 1, 8);
//  cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8, 0.8, 0.0, 1);
  char info_str[50];
  for (int i=0; i<moving_object_vector_->size(); i++)//遍历跟踪目标
  {
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    cv::Point temp_point4[4];
    for (int m = 0; m < 4; ++m)
    {
      CvPoint2D32f temp_point_f = cvPoint2D32f(temp_moving_object.contour_rect_pre.point4[m].x,
          temp_moving_object.contour_rect_pre.point4[m].y);
      CvPoint temp_point;
      BaseFunction::point_veh_to_img(img_result, &temp_point, temp_point_f, map_resolution, map_offset_y);
      temp_point4[m] = temp_point;
    }
    const cv::Point* ppt = temp_point4;

    int point_count = 4;
    //跟踪目标周围显示目标信息
    cv::polylines(img_test,&ppt,&point_count,1,true,cv::Scalar(255,0,255));//目标矩形框//洋红色

    sprintf(file_name_ID, "%d", i);
    word_point = cvPoint(temp_point4[0].x+2,temp_point4[0].y);
    cv::putText(img_test,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(0,255,255));//目标ID号黄色

    //图像左上方显示匹配信息
    sprintf(info_str,"%d %d %.2f %d %d %d %d %d %d",
        i,
        temp_moving_object.motion_behavior,
        temp_moving_object.motion_state_ab.v_post,
        match_state_mt_[i].is_match,
        match_state_mt_[i].match_index,
        temp_moving_object.track_state.tracked_times,
        temp_moving_object.track_state.tracked_times_c,
        temp_moving_object.track_state.missed_times_c,
        temp_moving_object.track_state.tracked_state);
    word_point = cvPoint(5,65+i*10);
    cv::putText(img_test,info_str,word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,0,255));
  }//end for (int i=0; i<moving_object_vector_->size(); i++)

  /***************************
   * 绘制检测目标及显示检测目标信息
   ***************************/
  if (0)
  {
    char info_str[50];
    for (int i=0; i<candidate_object_vector->size(); i++)//遍历当前检测到目标
    {
      CandidateObject temp_object = candidate_object_vector->at(i);
      CvPoint temp_point;
      BaseFunction::point_veh_to_img(img_result, &temp_point, temp_object.center_point, map_resolution, map_offset_y);
      sprintf(info_str,"%d %d",i, match_state_cd_[i].is_match);
      word_point = cvPoint(225,65+i*10);
      cv::putText(img_test,info_str,word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,125,0));

      sprintf(info_str,"%d",i);
      word_point = cvPoint(temp_point.x+2,temp_point.y);
      cv::circle(img_compare, temp_point, 2,cvScalar(0,255,0),1,8,0);//当前检测到目标有匹配上某个跟踪目标
      cv::putText(img_compare,info_str,word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,125,0));
    }
  }

  cv::namedWindow("moving_match_status",CV_WINDOW_NORMAL);
  cv::imshow("moving_match_status",img_test);
//
//  cv::namedWindow("candidate_match_status",CV_WINDOW_NORMAL);
//  cv::imshow("candidate_match_status",img_compare);
}
