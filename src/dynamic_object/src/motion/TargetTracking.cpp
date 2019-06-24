#include "TargetTracking.h"
#include <algorithm>
TrackingUpdate::TrackingUpdate():
time_stamp(0.0),
is_show_result_(true),
map_width(0),
map_height(0),
map_offset_y(0),
map_resolution(0.0),
virtual_line_num(360),
virtual_line_resolution(0.2)
{
}

TrackingUpdate::~TrackingUpdate()
{
}

void TrackingUpdate::Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
        const int& virtual_line_num_, const float& virtual_line_resolution_, bool is_show_result)
{
  map_width = boost::math::round(ogm_width_ / ogm_resolution_) +1;
  map_height = boost::math::round(ogm_height_ / ogm_resolution_) +1;
  map_resolution = ogm_resolution_;
  map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);

  virtual_line_num = virtual_line_num_;
  virtual_line_resolution = virtual_line_resolution_;
  this->is_show_result_ = is_show_result;

  motion_global.Init(ogm_width_, ogm_height_, ogm_offest_y_, ogm_resolution_, virtual_line_num_,
      virtual_line_resolution_);
  motion_traj.Init(ogm_width_, ogm_height_, ogm_offest_y_, ogm_resolution_, virtual_line_num_,
        virtual_line_resolution_);
}

void TrackingUpdate::Init(int map_width, int map_height, int map_offest_y,float map_resolution,
              int virtual_line_num_, float virtual_line_resolution_,bool is_show_result)
{
  this->map_width = map_width;//401
  this->map_height = map_height;//501
  this->map_offset_y = map_offest_y;//250
  this->map_resolution = map_resolution;//0.2

  this->virtual_line_num = virtual_line_num_; //360
  this->virtual_line_resolution = virtual_line_resolution_;//
  this->is_show_result_ = is_show_result;
}

void TrackingUpdate::MapInput(int frame_counter_,const double&  time_stamp_, const State_Vehicle& ego_veh_state_)
{
  time_stamp = time_stamp_;
  ego_veh_state_current = ego_veh_state_;
  motion_global.MapInput(frame_counter_,time_stamp_, ego_veh_state_);
  motion_traj.MapInput(frame_counter_,time_stamp_, ego_veh_state_);
}

void TrackingUpdate::Identify_PoseHead( MovingObject *moving_object_)
{
  if (moving_object_->shape.head_index<0)
  {
    float delata_length = moving_object_->shape.line_length0 - moving_object_->shape.line_length1;
    if(moving_object_->shape.is_entire_line0 && moving_object_->shape.is_entire_line1)
    {
      if(delata_length > 1)
        moving_object_->shape.head_index = 1;

      if(delata_length < -1)
        moving_object_->shape.head_index = 0;

    }
    if (moving_object_->shape.line_length0 > 4.0 || moving_object_->shape.line_length1>4.0)
    {
      if(moving_object_->shape.line_length0 >  moving_object_->shape.line_length1 && fabs(delata_length) >1)
        moving_object_->shape.head_index = 1;

      if(moving_object_->shape.line_length1 > moving_object_->shape.line_length0 && fabs(delata_length) >1)
        moving_object_->shape.head_index = 0;
    }
  }
  if (!moving_object_->shape.is_entire_head
      && moving_object_->motion_behavior > 0 && moving_object_->track_state.tracked_times>10)
  {
    CvPoint2D32f center_point = moving_object_->center_point;
    CvPoint2D32f contour_line4[4]; // 01, 12 23 30
    for (int m=0;m<4; m++)
    {
      int index_up = BaseFunction::value_in_threshod_int(m + 1, 0, 3);
      contour_line4[m].x = moving_object_->shape.point4[index_up].x - moving_object_->shape.point4[m].x;
      contour_line4[m].y = moving_object_->shape.point4[index_up].y - moving_object_->shape.point4[m].y;
    }

    float ego_angle = ego_veh_state_current.global_position.heading * pi/180;
    CvPoint2D32f point_v_32f;
    point_v_32f.x = moving_object_->motion_state_ab.rough_v_x*cos(ego_angle) - moving_object_->motion_state_ab.rough_v_y*sin(ego_angle);
    point_v_32f.y = moving_object_->motion_state_ab.rough_v_x*sin(ego_angle) + moving_object_->motion_state_ab.rough_v_y*cos(ego_angle);

    int head_index = 0;
    float min_delta_angle = 361;
    float delta_v_line[4];
    for (int m=0; m<4; m++)
    {
      float temp_angle = BaseFunction::Angle_line2(point_v_32f, contour_line4[m]);
      delta_v_line[m] = temp_angle;
      if (min_delta_angle > temp_angle)
      {
        min_delta_angle = temp_angle;
        head_index = BaseFunction::value_in_threshod_int(m + 1, 0, 3);
      }
    }
    if(min_delta_angle < 20)
    {
      if(moving_object_->shape.head_index>0)
      {
        if (head_index == moving_object_->shape.head_index || head_index == (moving_object_->shape.head_index +2))
        {
          moving_object_->shape.head_index = head_index;
          moving_object_->shape.is_entire_head = true;
        }
      }
      else
      {
        if(min_delta_angle < 10)
        {
          moving_object_->shape.head_index = head_index;
          moving_object_->shape.is_entire_head = true;
        }
      }
    }
  }
}

void TrackingUpdate::ObjectTypeClassify(MovingObject *moving_object_)
{
  //TODO:可以在该步骤加入一些目标剔除,即目标形状变化剧烈的,特别是面积发生突变的应该肯有可能是背景
  int temp_object_type = moving_object_->object_type;

  float object_width = moving_object_->shape.line_length0;
  float object_length = moving_object_->shape.line_length1;
  float tmep_object_high = moving_object_->shape.object_height;
  object_width = min(object_width, object_length);
  object_length = max(object_width, object_length);

  if (object_length <= 0.7)
  {
    temp_object_type = 1;
    if (object_width>0.7 && object_width<=1.3)
      temp_object_type = 2;

    if (object_width>1.3 && object_width<=2.5)
      temp_object_type = 3;

    if (object_width>2.5 && object_width<=4.0)
      temp_object_type = 4;
  }
  if (object_length>0.7 && object_length<=1.3)
  {
    temp_object_type = 2;
    if (object_width>1.3 && object_width<=2.5)
      temp_object_type = 3;

    if (object_width>2.5 && object_width<=4.0)
      temp_object_type = 4;
  }
  if (object_length>1.3 && object_length<=2.5)
  {
    temp_object_type = 3;
    if (object_width>2.5 && object_width<=4.0)
      temp_object_type = 4;

  }
  if (object_length>2.5 && object_length<=4.0)
    temp_object_type = 4;

  if (object_length > 4.0 || object_width>4.0)
  {
    moving_object_->shape.is_entire_type = true;
    temp_object_type = 5;
    if (object_length>4.0 && object_length<=6.5)
      temp_object_type = 5;

    if (object_length>6.5 && object_length<=8.5 )
      temp_object_type = 6;// 6

      if (object_length>8.5 && object_length<=12.5 )
        temp_object_type = 7;

      if (object_length>12.5 && object_length<=18.5 )
        temp_object_type = 8;

      if (object_length>18.5)
        temp_object_type = 9;
  }
  if(moving_object_->object_type<0)
    temp_object_type = abs(temp_object_type);
  moving_object_->object_type = temp_object_type;
}

void TrackingUpdate::UpdateHistoryInfo(MovingObject* moving_object_){
  HistoryInfo history_info;
  history_info.time_stamp = moving_object_->timestamp;
  history_info.history_center_ab = moving_object_->center_pt_meas_ab;
  history_info.history_rect_ab = moving_object_->contour_rect_ab;

  moving_object_->history_info.push_back(history_info);//加入历史信息
  if(moving_object_->history_info.size() > HISTORY_NUM){
    moving_object_->history_info.pop_front();//删除历史第一个
  }
}

void TrackingUpdate::MotionStateUpdating(vector<CandidateObject>* candidat_object_vector_, vector<MovingObject> *moving_object_vector_)
{
  for (int i = 0; i < moving_object_vector_->size(); ++i) {
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    //运动状态更新
    motion_global.MotionState_Filter_ab(&temp_moving_object);
    //根据矩形框几何特征区分物体类别
    ObjectTypeClassify(&temp_moving_object);//更新完之后再次判别物体类别
    if (abs(temp_moving_object.object_type) > 1)
      Identify_PoseHead(&temp_moving_object);
//    //zhanghm change: 20180610
//    if (temp_moving_object.is_updated && temp_moving_object.track_state.tracked_times>0){//TODO:有匹配目标才更新历史状态？
//      Update_history_state( &temp_moving_object );
//    }
    motion_traj.UpdateMotionBehavior(&temp_moving_object);
    UpdateHistoryInfo(&temp_moving_object);//更新历史信息
    moving_object_vector_->at(i) = temp_moving_object;
  }//end for (int i = 0; i < (moving_object_vector_->size()); i++)

  for (int i = 0; i < moving_object_vector_->size(); i++){//对跟踪目标确定匹配范围,用于下一次匹配分析用
    MovingObject temp_moving_object = moving_object_vector_->at(i);
    //划定匹配区域,用于下一次匹配分析
    temp_moving_object.match_range =Search_matchrange( img_ogm, temp_moving_object);
    //	  //TODO：这是在干嘛？object_type到底起到什么作用了？
    //	  if (temp_moving_object.candidate_index>-1)//有匹配目标
    //	  {
    //	    int index_cd = temp_moving_object.candidate_index;
    //	    CandidateObject temp_candidate = candidat_object_vector_->at(index_cd);
    //	    if (temp_candidate.object_type<0 && temp_moving_object.object_type>0)
    //	    {
    //	      temp_moving_object.object_type = -temp_moving_object.object_type;
    //	    }
    //	  }
    moving_object_vector_->at(i) = temp_moving_object;
  }
}

MatchRange_grid TrackingUpdate::Search_matchrange( IplImage* img_, MovingObject moving_object_ )
{
  int object_type = abs(moving_object_.object_type);

  CvPoint contour_point4[4];
  CvPoint2D32f contour_point4_f[4];
  CvPoint2D32f temp_center_f;
  Line_s contour_line4[4];
  if (moving_object_.candidate_index>-1)
  {
    for (int m = 0; m < 4; ++m)
    {
      CvPoint2D32f temp_point;
      temp_point.x = moving_object_.shape.point4[m].x/map_resolution + map_width/2;
      temp_point.y = map_height-1 - moving_object_.shape.point4[m].y/map_resolution -map_offset_y;
      contour_point4[m] = cvPointFrom32f(temp_point);
      contour_point4_f[m] = temp_point;
    }
    temp_center_f.x = moving_object_.center_point.x/map_resolution + map_width/2;
    temp_center_f.y = map_height-1 - moving_object_.center_point.y/map_resolution - map_offset_y;
  }
  else {
    for (int m = 0; m < 4; ++m)
    {
      CvPoint2D32f temp_point;
      temp_point.x = moving_object_.contour_rect_pre.point4[m].x/map_resolution + map_width/2;
      temp_point.y = map_height-1 - moving_object_.contour_rect_pre.point4[m].y/map_resolution -map_offset_y;
      contour_point4[m] = cvPointFrom32f(temp_point);
      contour_point4_f[m] = temp_point;
    }
    temp_center_f.x = moving_object_.center_point_pre.x/map_resolution + map_width/2;
    temp_center_f.y = map_height-1 - moving_object_.center_point_pre.y/map_resolution - map_offset_y;
  }

  for (int m = 0; m < 4; ++m)
  {
    int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
    float temp_x_up = contour_point4_f[m_up].x -  contour_point4_f[m].x;
    float temp_y_up = contour_point4_f[m_up].y -  contour_point4_f[m].y;
    contour_line4[m].angle = atan2(temp_y_up, temp_x_up);
    contour_line4[m].line_length = sqrt(temp_x_up*temp_x_up + temp_y_up*temp_y_up);
  }

  MatchRange_grid match_range = moving_object_.match_range;
  bool is_search = false;
  float temp_v_x = moving_object_.motion_state_ab.rough_v_x;
  float temp_v_y = moving_object_.motion_state_ab.rough_v_y;
  float temp_v = sqrt(temp_v_x*temp_v_x + temp_v_y*temp_v_y);

  if (moving_object_.track_state.tracked_times < 5 || (temp_v > 1))
    is_search = true;
  if (is_search)
  {
    CvPoint2D32f line_point4[4];
    CvPoint temp_center = cvPointFrom32f(temp_center_f);
    for (int m = 0; m < 4; ++m)
    {
      int m_pre = BaseFunction::value_in_threshod_int(m-1 , 0, 3);
      float temp_angle_pre = contour_line4[m_pre].angle;
      float temp_angle = contour_line4[m].angle;
      float temp_length_up = contour_line4[m].line_length;
      float temp_length_pre = contour_line4[m_pre].line_length;

      float temp_dis = 3;
      float max_dis = Search_dis(moving_object_, m);

      CvPoint2D32f start_point = contour_point4_f[m];
      if (moving_object_.object_type>2)
      {
        for (int i=4; i<max_dis; i++)
        {
          temp_dis = i;
          start_point.x = contour_point4[m].x + i*cos(temp_angle_pre);
          start_point.y = contour_point4[m].y + i*sin(temp_angle_pre);

          CvPoint2D32f temp_point_f = start_point;
          bool is_break = false;
          for ( int j=0; j<temp_length_up+1; j++)
          {
            temp_point_f.x = start_point.x + j*cos(temp_angle);
            temp_point_f.y = start_point.y + j*sin(temp_angle);
            CvPoint temp_point = cvPointFrom32f(temp_point_f);

            if (temp_point.x<1 || temp_point.x> (img_->width-1)
                || temp_point.y<1 || temp_point.y> (img_->height -2) )
            {
              is_break = true;
              break;
            }
            if(0)// (temp_point.x>0 && temp_point.x<img_result->width && temp_point.y>0 && temp_point.y<img_result->height)
                            {
              ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 0]=125; // B
              ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 1]=125; // G
              ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 2]=125; // R

                            }
            uchar* val1 = ((uchar *)(img_->imageData + temp_point.y*img_->widthStep));
            if (val1[temp_point.x * img_->nChannels] > 0)
            {
              is_break = true;
              break;
            }
          }
          if (is_break)
            break;
        }
      } else
      {
        temp_dis = 2/map_resolution;
      }

      line_point4[m] = start_point;
      float tem_length_dis = temp_dis + temp_length_pre/2;
      match_range.range_dis[m] = tem_length_dis*map_resolution;
    }
    match_range.d_radius = 5;
    if (object_type < 3)
      match_range.d_radius = 3;
  }

  CvPoint2D32f center_point;
  center_point.x = (temp_center_f.x - map_width/2)*map_resolution;
  center_point.y = (map_height-1 - temp_center_f.y - map_offset_y)*map_resolution;
  for (int m = 0; m < 4; ++m)
  {
    int m_pre = BaseFunction::value_in_threshod_int(m-1 , 0, 3);
    float temp_angle_pre = contour_line4[m_pre].angle;
    match_range.range_rect4[m].x = center_point.x + match_range.range_dis[m]*cos(temp_angle_pre);
    match_range.range_rect4[m].y = center_point.y - match_range.range_dis[m]*sin(temp_angle_pre);
  }
  if (1)
  {
    CvPoint center_point = cvPointFrom32f(temp_center_f);
    if (object_type < 2)
      cvCircle(img_result, center_point, match_range.d_radius/map_resolution, cvScalar(0,255,0), 1);

    CvPoint temp_point4[4];
    for (int m = 0; m < 4; ++m)
    {
      CvPoint2D32f temp_point;
      temp_point.x = match_range.range_rect4[m].x/map_resolution + img_->width/2;
      temp_point.y = img_->height-1 - match_range.range_rect4[m].y/map_resolution - map_offset_y;
      temp_point4[m] = cvPointFrom32f(temp_point);
    }
    CvPoint* ppt_test = temp_point4;
    int point_count = 4;
    cvPolyLine(img_result, &ppt_test, &point_count, 1, 1, cvScalar(0,255,255), 1);
  }
  return match_range;
}


float TrackingUpdate::Search_dis(MovingObject moving_object_, int index_)
{
  int object_type = abs(moving_object_.object_type);
  float max_dis = 3/map_resolution;
  if (moving_object_.track_state.tracked_times < 2)
  {
    float max_dis = 4/map_resolution;
    if(moving_object_.shape.head_index > -1 && object_type > 3)
    {
      if( ( index_ == moving_object_.shape.head_index || index_ == (moving_object_.shape.head_index+2)
          || index_ == (moving_object_.shape.head_index-2) ) )
        max_dis = 8/map_resolution;

      if( !( index_ == moving_object_.shape.head_index || index_ == (moving_object_.shape.head_index+2)
          || index_ == (moving_object_.shape.head_index-2) ) )
        max_dis = 2/map_resolution;
    }
  }

  if (moving_object_.track_state.tracked_times > 3)
  {
    float ego_angle = ego_veh_state_current.global_position.heading*pi/180;
    float temp_v_x = moving_object_.motion_state_ab.rough_v_x*cos(ego_angle) - moving_object_.motion_state_ab.rough_v_y*sin(ego_angle);
    float temp_v_y = moving_object_.motion_state_ab.rough_v_x*sin(ego_angle) + moving_object_.motion_state_ab.rough_v_y*cos(ego_angle);
    float temp_v = sqrt(temp_v_x*temp_v_x + temp_v_y*temp_v_y);
    if (moving_object_.shape.head_index > -1)
    {
      if( ( index_ == moving_object_.shape.head_index || index_ == (moving_object_.shape.head_index+2)
          || index_ == (moving_object_.shape.head_index-2) ) )
        max_dis = 8/map_resolution;

      if( !( index_ == moving_object_.shape.head_index || index_ == (moving_object_.shape.head_index+2)
          || index_ == (moving_object_.shape.head_index-2) ) )
        max_dis = 2/map_resolution;
    }

    if (moving_object_.track_state.missed_times_c > 0){
      //zhanghm 20180630 change delta_time to below
//      float delta_time = ego_veh_state_current.time_stamp - moving_object_.history_state.history_time[0];
      auto iter_end = moving_object_.history_info.rbegin()+1;
      float delta_time = ego_veh_state_current.time_stamp - iter_end->time_stamp;
      max_dis = max_dis + (delta_time*temp_v)/map_resolution;
      max_dis = min(10/map_resolution, max_dis);
    }
  }
  return max_dis;
}

void TrackingUpdate::Target_Array( vector<MovingObject> *moving_object_vector_ )
{
  vector<MovingObject> temp_moving_object_total;
  temp_moving_object_total.clear();

  if (1)//TODO:此步骤在干嘛？为什么需要分为7和不是7讨论
  {
    //zhanghm add:20180608
    //按角度从小到大排序
    std::sort(moving_object_vector_->begin(),moving_object_vector_->end(),
        [](const MovingObject& obj1,const MovingObject& obj2){return obj1.position_angle<obj2.position_angle?true:false;});
    return;
  }
  else
  {
    float temp_region_anle_resolution = 1.0;
    int temp_region_cell_size = boost::math::round(360 / temp_region_anle_resolution);
    Cluster_Index* moving_object_region = new Cluster_Index[temp_region_cell_size];

    for (int i=0; i<temp_region_cell_size; i++)
    {
      moving_object_region[i].index_num = 0;
      moving_object_region[i].index_vector.clear();
    }

    for (int m=0; m<moving_object_vector_->size(); m++)
    {
      MovingObject temp_object = moving_object_vector_->at(m);
      int region_index = (int)( temp_object.position_angle / temp_region_anle_resolution);
      if ( region_index > (temp_region_cell_size-1) )
      {
        region_index = temp_region_cell_size - 1;
      }
      moving_object_region[region_index].index_vector.push_back(m);
    }
    for (int i=0; i<temp_region_cell_size; i++)
    {
      if (moving_object_region[i].index_vector.size() > 0)
      {
        for (int m=0; m < moving_object_region[i].index_vector.size(); m++)
        {
          int candidate_index = moving_object_region[i].index_vector.at(m);
          MovingObject temp_object = moving_object_vector_->at(candidate_index);
          temp_moving_object_total.push_back( temp_object );
        }
      }
    }
    delete[] moving_object_region;
  }
  moving_object_vector_->clear();
  for (int i=0; i<temp_moving_object_total.size(); i++)
  {
    MovingObject temp_moving_object = temp_moving_object_total.at(i);
    moving_object_vector_->push_back(temp_moving_object);
  }
  if (moving_object_vector_->size() < 7)//TODO:此步骤在干嘛？为什么需要分为7和不是7讨论
  {
    std::cout<<"after array..."<<std::endl;
    for(auto obj:temp_moving_object_total)
    {
      std::cout<<obj.position_angle<<" "<<std::endl;
    }
  }
}

void TrackingUpdate::Release()
{
  cvReleaseImage(&img_ogm);
  cvReleaseImage(&img_result);
}

void TrackingUpdate::UpdateMotionState( IplImage *img_ogm_,
                                        vector<CandidateObject>* candidat_object_vector_,
                                        vector<MovingObject>* moving_object_vector_ )
{
  cout<<"[TrackingMotionUpdate] enter in TrackingUpdate::UpdateMotionState..."<<endl;
  img_ogm = cvCreateImage(cvGetSize(img_ogm_), 8, 1);
  img_result = cvCreateImage(cvGetSize(img_ogm_), 8, 3);
  cvCopy(img_ogm_, img_ogm);
  cvCvtColor(img_ogm_, img_result, CV_GRAY2BGR);

  MotionStateUpdating( candidat_object_vector_, moving_object_vector_ );
  Target_Array( moving_object_vector_ );

  //可视化
  if (this->is_show_result_)
  {
    cvNamedWindow("img_result_track",CV_WINDOW_NORMAL);
    cvShowImage("img_result_track",img_result);
    cvWaitKey(5);
  }
  Release();
}


