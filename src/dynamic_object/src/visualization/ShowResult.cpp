#include <visualization/ShowResult.h>
#include <sstream>
using std::cout; using std::endl;
ShowResult::ShowResult()
{
}
ShowResult::~ShowResult()
{
}

void ShowResult::Init(float ogm_resolution_, float ogm_offest_y_)
{
  map_resolution = ogm_resolution_;
  map_offset_y = ogm_offest_y_/ map_resolution;
}

void ShowResult::ShowEgoVehicle(IplImage* img_, int map_offest_y_, float ogm_resolution_, float egocar_with_, float egocar_height_)
{
  CvPoint ego_vehicle = cvPoint( img_->width/2, (int)(img_->height - map_offest_y_) );
  int vehicle_width = egocar_with_/ ogm_resolution_;
  int vehicle_height = egocar_height_/ ogm_resolution_;
  CvPoint ego_vehicle_lt = cvPoint(ego_vehicle.x - vehicle_width/2, ego_vehicle.y - vehicle_height/2);
  CvPoint ego_vehicle_rb = cvPoint(ego_vehicle.x + vehicle_width/2, ego_vehicle.y + vehicle_height/2);
  cvRectangle(img_,ego_vehicle_lt,ego_vehicle_rb,cvScalar(0,0,0),-1,8,0);

  int tire_width = 1/ogm_resolution_;
  int tire_height = 2/ogm_resolution_;

  CvPoint ego_vehicle_ltb = cvPoint(ego_vehicle_lt.x + tire_width/2,ego_vehicle_lt.y + tire_height/2);
  cvRectangle(img_,ego_vehicle_lt,ego_vehicle_ltb,cvScalar(0,255,255),-1,8,0);

  CvPoint ego_vehicle_rtt = cvPoint(ego_vehicle.x + vehicle_width/2,ego_vehicle.y - vehicle_height/2);
  CvPoint ego_vehicle_rtb = cvPoint(ego_vehicle_rtt.x - tire_width/2,ego_vehicle_rtt.y + tire_height/2);
  cvRectangle(img_,ego_vehicle_rtt,ego_vehicle_rtb,cvScalar(0,255,255),-1,8,0);
  CvPoint ego_vehicle_rbt = cvPoint(ego_vehicle_rb.x - tire_width/2,ego_vehicle_rb.y - tire_height/2);
  cvRectangle(img_,ego_vehicle_rb,ego_vehicle_rbt,cvScalar(0,255,255),-1,8,0);

  CvPoint ego_vehicle_lbb = cvPoint(ego_vehicle.x - vehicle_width/2,ego_vehicle.y + vehicle_height/2);
  CvPoint ego_vehicle_lbt = cvPoint(ego_vehicle_lbb.x + tire_width/2,ego_vehicle_lbb.y - tire_height/2);
  cvRectangle(img_,ego_vehicle_lbb,ego_vehicle_lbt,cvScalar(0,255,255),-1,8,0);

  CvPoint pt[3];
  pt[0] = cvPoint(ego_vehicle.x,ego_vehicle.y - 1.2/ogm_resolution_);
  pt[1] = cvPoint(ego_vehicle.x - 0.6/ogm_resolution_, ego_vehicle.y + 0.8/ogm_resolution_);
  pt[2] = cvPoint(ego_vehicle.x + 0.6/ogm_resolution_, ego_vehicle.y + 0.8/ogm_resolution_);

  cvFillConvexPoly(img_,pt,3,cvScalar(0,255,0),8,0);

  int dis_10 = boost::math::round(10/ogm_resolution_);
  int dis10_width_num = 3;
  int dis10_height_num = 3;
  for (int i=0; i<(dis10_height_num); i++)
  {
    CvPoint line_point0;
    CvPoint line_point1;
    line_point0.x = 0;
    line_point0.y = ego_vehicle.y + i*20/ogm_resolution_;
    line_point1.x = img_->width;
    line_point1.y = ego_vehicle.y + i*20/ogm_resolution_;
    cvLine(img_, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
    if(1)
    {
      CvFont font;
      cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);
      char file_name_ID[50];
      sprintf(file_name_ID, "%dm ", -i*20);
      CvScalar word_color = cvScalar(125,125,125);
      CvPoint word_point;
      word_point.x = line_point0.x +2;
      word_point.y = line_point0.y;
      cvPutText(img_, file_name_ID, word_point, &font, word_color);
    }
    if (i>0)
    {
      line_point0.y = ego_vehicle.y - i*20/ogm_resolution_;
      line_point1.y = ego_vehicle.y - i*20/ogm_resolution_;
      cvLine(img_, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
      if(1)
      {
        CvFont font;
        cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);
        char file_name_ID[50];
        sprintf(file_name_ID, "%dm ", i*20);
        CvScalar word_color = cvScalar(125,125,125);
        CvPoint word_point;
        word_point.x = line_point0.x +5;
        word_point.y = line_point0.y;
        cvPutText(img_, file_name_ID, word_point, &font, word_color);
      }
    }
  }
  for (int i=0; i<(dis10_width_num); i++)
  {
    CvPoint line_point0;
    CvPoint line_point1;

    line_point0.x = ego_vehicle.x + i*20/ogm_resolution_ + 10/ogm_resolution_;
    line_point0.y = 0;
    line_point1.x = ego_vehicle.x + i*20/ogm_resolution_ + 10/ogm_resolution_;
    line_point1.y = img_->height;
    cvLine(img_, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);

    line_point0.x = ego_vehicle.x - i*20/ogm_resolution_ - 10/ogm_resolution_;
    line_point1.x = ego_vehicle.x - i*20/ogm_resolution_ - 10/ogm_resolution_;
    cvLine(img_, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
    if (i==0)
    {
      line_point0.x = ego_vehicle.x;
      line_point0.y = ego_vehicle.y + 10/ogm_resolution_;
      line_point1.x = ego_vehicle.x;
      line_point1.y = ego_vehicle.y - 10/ogm_resolution_;
      cvLine(img_, line_point0, line_point1, cvScalar(155,155,155), 1, 8, 0);
    }
  }
}

void ShowResult::ShowPolarOGM(IplImage* img_, Polar_Cell* polar_ogm_, float polarogm_angle_, float polarogm_radius_,
    float polarogm_angle_resolution_, float polarogm_radius_resolution_)
{
  int polarogm_angle_cell_ = boost::math::round(polarogm_angle_ / polarogm_angle_resolution_);
  int polarogm_radius_cell_ = boost::math::round((polarogm_radius_-10) / polarogm_radius_resolution_);;
  for ( int j=0; j<polarogm_angle_cell_; j++) {//遍历周向
    float angle0 = j*polarogm_angle_resolution_;
    float angle1 = (j+1)*polarogm_angle_resolution_;
    float angle2 = (j+0.5)*polarogm_angle_resolution_;
    angle0 = 2*pi - angle0*pi/180;
    angle1 = 2*pi - angle1*pi/180;
    angle2 = 2*pi - angle2*pi/180;
    float dis_line = (polarogm_angle_cell_-0.5)*polarogm_radius_resolution_;
    for (int i=7; i<polarogm_radius_cell_; i++)
    {
      int polar_index = i*polarogm_angle_cell_ + j;
      float dis_xy0 = i*polarogm_radius_resolution_;
      float dis_xy1 = (i+1)*polarogm_radius_resolution_;
      float dis_xy2 = (i+0.5)*polarogm_radius_resolution_;
      CvPoint2D32f point_32[4];
      point_32[0].x = dis_xy0*cos(angle0)/map_resolution + img_->width/2;
      point_32[0].y = img_->height - dis_xy0*sin(angle0)/map_resolution - img_->height/2;
      point_32[1].x = dis_xy1*cos(angle0)/map_resolution + img_->width/2;
      point_32[1].y = img_->height - dis_xy1*sin(angle0)/map_resolution - img_->height/2;
      point_32[2].x = dis_xy1*cos(angle1)/map_resolution + img_->width/2;
      point_32[2].y = img_->height - dis_xy1*sin(angle1)/map_resolution - img_->height/2;
      point_32[3].x = dis_xy0*cos(angle1)/map_resolution + img_->width/2;
      point_32[3].y = img_->height - dis_xy0*sin(angle1)/map_resolution - img_->height/2;

      //CvPoint pt[4];
      //for (int m=0; m<4; m++)
      //{
      //	pt[m] = cvPointFrom32f(point_32[m]);
      //}
      if (static_cast<int>(polar_ogm_[polar_index].type) > static_cast<int>(CellProperty::Unknown))
      {
        //cvFillConvexPoly(img_,pt,4,cvScalar(0,255,0),8,0);
        if (polar_ogm_[polar_index].type == CellProperty::RigidNoPassable)
        {
          //cvFillConvexPoly(img_,pt,4,cvScalar(0,255,0),8,0);
          //cvLine(img_, line_point[0], line_point[1], cvScalar(0,255,0), 1, 8, 0);
          dis_line = (i+0.5)*polarogm_radius_resolution_;
          break;
        }
      }
    }
    if (1)
    {
      CvPoint2D32f line_point_32[2];
      line_point_32[0].x = img_->width/2;
      line_point_32[0].y = img_->height/2;
      line_point_32[1].x = dis_line*cos(angle2)/map_resolution + img_->width/2;
      line_point_32[1].y = img_->height - dis_line*sin(angle2)/map_resolution - img_->height/2;
      CvPoint line_point[2];
      line_point[0] = cvPointFrom32f(line_point_32[0]);
      line_point[1] = cvPointFrom32f(line_point_32[1]);
      cvLine(img_, line_point[0], line_point[1], cvScalar(0,255,0), 1, 8, 0);
    }
  }
}

void ShowResult::ShowObject_candidate(IplImage* img_, CandidateObject temp_object , int object_ID_,
    CvScalar object_color_, int thinkness_)
{
  int point_count = 4;
  CvPoint contour_point4[4];
  CvPoint center_point;
  center_point.x = (int)(temp_object.center_point.x/map_resolution + img_->width/2);
  center_point.y = (int)(img_->height - temp_object.center_point.y/map_resolution - img_->height/2);

  for(int i=0; i<4; i++)
  {
    CvPoint2D32f temp_point;
    temp_point.x = (temp_object.shape.point4[i].x/map_resolution + img_->width/2);
    temp_point.y = (img_->height-1 - temp_object.shape.point4[i].y/map_resolution - map_offset_y);
    contour_point4[i] = cvPointFrom32f(temp_point);
  }
  if (1)
  {
    CvPoint* line_point4 = contour_point4;
    CvScalar object_color = cvScalar(255,0,0);
    cvPolyLine(img_, &line_point4, &point_count, 1, 1, object_color, thinkness_);
  }
  if(0)
  {
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);

    char file_name_ID[50];
    float temp_pose_head = temp_object.object_box.angle;
    //sprintf(file_name_ID, "%d %.2f %.2f", object_ID_, temp_object.center_point.x, temp_object.center_point.y);
    //sprintf(file_name_ID, "%.1f ", temp_object.pose_heading);
    sprintf(file_name_ID, "%.1f %.1f ", temp_object.center_point.x, temp_object.center_point.y);
    //
    CvScalar word_color = cvScalar( 0, 0, 255 );
    CvPoint word_point;
    word_point.x = temp_object.object_rect.x + temp_object.object_rect.width;
    word_point.y = temp_object.object_rect.y;
    cvPutText(img_, file_name_ID, word_point, &font, word_color);
  }
  if (0)
  {
    int track_index0 = temp_object.track_index0;
    int track_index1 = temp_object.track_index1;
    CvPoint2D32f track_point0_32, track_point1_32, track_point_32;
    track_point0_32.x = (temp_object.shape.point4[track_index0].x/map_resolution + img_->width/2);
    track_point0_32.y = (img_->height - temp_object.shape.point4[track_index0].y/map_resolution - map_offset_y);
    track_point1_32.x = (temp_object.shape.point4[track_index1].x/map_resolution + img_->width/2);
    track_point1_32.y = (img_->height - temp_object.shape.point4[track_index1].y/map_resolution - map_offset_y);
    track_point_32.x = (temp_object.track_point.x/map_resolution + img_->width/2);
    track_point_32.y = (img_->height - temp_object.track_point.y/map_resolution - map_offset_y);

    CvPoint track_point0, track_point1, track_point;
    track_point0 = cvPointFrom32f(track_point0_32);
    track_point1 = cvPointFrom32f(track_point1_32);
    track_point = cvPointFrom32f(track_point_32);
    cvCircle(img_, track_point, 2, cvScalar(0,0,255), 2, 8, 0);

  }

}
void ShowResult::ShowObject_moving(IplImage* img_, MovingObject temp_object, const State_Vehicle& ego_veh_state_current)
{
  CvPoint contour_point4[4];
  CvPoint center_point;
  //得到目标中心点像素坐标
  center_point.x = (int)(temp_object.center_pt.x/map_resolution + img_->width/2);
  center_point.y = (int)(img_->height-1 - temp_object.center_pt.y/map_resolution - img_->height/2);
  //根据目标是否有匹配,计算不同的contour_point4
//  if(temp_object.has_match){
//    for(int i=0; i<4; i++){
//      CvPoint2D32f temp_point;
//      temp_point.x = (temp_object.shape.point4[i].x/map_resolution + img_->width/2);
//      temp_point.y = (img_->height-1 - temp_object.shape.point4[i].y/map_resolution - map_offset_y);
//      contour_point4[i] = cvPointFrom32f(temp_point);
//    }
//  }
//  else{
//    for(int i=0; i<4; i++){
//      CvPoint2D32f temp_point;
//      temp_point.x = (temp_object.contour_rect_pre.point4[i].x/map_resolution + img_->width/2);
//      temp_point.y = (img_->height-1 - temp_object.contour_rect_pre.point4[i].y/map_resolution - map_offset_y);
//      contour_point4[i] = cvPointFrom32f(temp_point);
//    }
//    center_point.x = (contour_point4[0].x + contour_point4[2].x)/2;
//    center_point.y = (contour_point4[0].y + contour_point4[2].y)/2;
//  }

  //矩形框四个点转像素坐标
  for ( int m=0; m<4; m++){
    CvPoint2D32f temp_p;
    Point_d temp_point_g = temp_object.contour_rect_ab.point4[m];
    Point2D temp_point;
    //转车体坐标
    BaseFunction::point_global_to_local(temp_point_g.x, temp_point_g.y, ego_veh_state_current, &temp_point.x, &temp_point.y);
    temp_p.x = (temp_point.x/map_resolution + img_->width/2);
    temp_p.y = (img_->height-1 - temp_point.y/map_resolution - map_offset_y);
    contour_point4[m] = cvPointFrom32f(temp_p);
  }
  center_point.x = (contour_point4[0].x + contour_point4[2].x)/2;
  center_point.y = (contour_point4[0].y + contour_point4[2].y)/2;


  int point_count = 4;
  if (1)//绘制目标矩形框
  {
    CvPoint* line_point4 = contour_point4;
    CvScalar object_color = cvScalar(0,255,255);//显示黄色
    if (!temp_object.has_match)
      object_color = cvScalar(0,0,255);//目标未匹配,显示红色
    cvPolyLine(img_, &line_point4, &point_count, 1, 1, object_color, 1);
    //cvLine(img_, contour_point4[0], contour_point4[1], cvScalar(0,255,255), 2, 8, 0);
//    cvCircle(img_, contour_point4[0], 2,cvScalar(0,255,0), 2, 8, 0);
//    cvCircle(img_, contour_point4[1], 2,cvScalar(0,255,255), 2, 8, 0);
//    cvCircle(img_, contour_point4[2], 2,cvScalar(255,255,0), 2, 8, 0);
//    cvCircle(img_, contour_point4[3], 2,cvScalar(255,0,0), 2, 8, 0);

  }

  if(1){//绘制圆区域查看周围障碍物情况
    int distance = floor(3.0/0.2);
    cvCircle(img_,center_point,distance,cvScalar(0,255,0),1);
  }

  if(0)//显示车头方向点
  {
    if (temp_object.shape.head_index>-1)
    {
      int head_index = temp_object.shape.head_index;
      int head_pre =  BaseFunction::value_in_threshod_int(head_index -1, 0, 3);
      int head_up = BaseFunction::value_in_threshod_int(head_index + 1, 0, 3);
      int head_diag = 6 - head_index - head_pre - head_up;
      CvPoint head_point;
      head_point.x = (contour_point4[head_index].x + contour_point4[head_up].x)/2;
      head_point.y = (contour_point4[head_index].y + contour_point4[head_up].y)/2;
      cvCircle(img_, contour_point4[head_index], 1,cvScalar(0,255,0), 2, 8, 0);
      cvLine(img_, contour_point4[head_index], contour_point4[head_pre], cvScalar(0,255,0), 2, 8, 0);
    }
  }

  if (0)//显示文字
  {
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);

    char file_name_ID[50];
    int temp_times = temp_object.track_state.tracked_times;
    if(!temp_object.is_updated)
    {
      temp_times = -temp_object.track_state.missed_times_c;
    }
    sprintf(file_name_ID, "%.2f %.2f",temp_object.shape.line_length0, temp_object.shape.line_length1);
    CvScalar word_color = cvScalar( 0, 0, 255 );
    CvPoint word_point;
    word_point.x = temp_object.object_rect.x + temp_object.object_rect.width/2;
    word_point.y = temp_object.object_rect.y;
    cvPutText(img_, file_name_ID, word_point, &font, word_color);

  }


  if (0)//显示相对运动速度
  {
    float temp_angle = ego_veh_state_current.global_position.heading*pi/180;
    CvPoint temp_v_s;
    CvPoint temp_v_e;
    CvPoint temp_v4_s[4];
    CvPoint temp_v4_e[4];
    float temp_v_x = temp_object.motion_state.v_x_post;
    float temp_v_y = temp_object.motion_state.v_y_post;
    CvPoint2D32f temp_point_f = temp_object.center_point;

    temp_point_f.x = (temp_point_f.x/map_resolution + img_->width/2);
    temp_point_f.y = (img_->height-1 - temp_point_f.y/map_resolution - map_offset_y);
    temp_v_s = cvPointFrom32f(temp_point_f);

    temp_point_f.x = temp_point_f.x + (temp_v_x)/1.5;
    temp_point_f.y = temp_point_f.y - (temp_v_y)/1.5;
    temp_v_e = cvPointFrom32f(temp_point_f);

    for (int m = 0; m < 4; ++m)
    {
      temp_point_f.x = (temp_object.shape.point4[m].x/map_resolution + img_->width/2);
      temp_point_f.y = (img_->height-1 - temp_object.shape.point4[m].y/map_resolution - map_offset_y);
      temp_v4_s[m] = cvPointFrom32f(temp_point_f);
      temp_v_x = temp_object.motion_state.v_x_pt4[m];
      temp_v_y = temp_object.motion_state.v_y_pt4[m];

      temp_point_f.x = temp_point_f.x + (temp_v_x)/map_resolution/1.5;
      temp_point_f.y = temp_point_f.y - (temp_v_y)/map_resolution/1.5;
      temp_v4_e[m] = cvPointFrom32f(temp_point_f);
    }
    cvLine(img_, temp_v_e, temp_v_s, cvScalar(175,0, 175), 1,8,0);

    if(0)
    {
      CvFont font;
      cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);

      char file_name_ID[50];
      memset(file_name_ID, 0, 50);
      float temp_v = 0;//sqrt(temp_x*temp_x + temp_y*temp_y)*1.1 + 0.5;
      //sprintf(file_name_ID, "%d", temp_object.track_state.tracked_times);
      sprintf(file_name_ID, "%d %d %.1f",temp_object.ID_number,temp_object.track_state.tracked_times, temp_v);
      //sprintf(file_name_ID, "%.1f %.1f", temp_v, temp_object.motion_state.pose_head_post*180/pi);
      //sprintf(file_name_ID, "%.1f ", temp_v*map_resolution*3.6 - ego_veh_state_current.fForwardVel*3.6);
      CvScalar word_color = cvScalar( 0, 0, 255 );
      CvPoint word_point;
      word_point.x = temp_object.object_rect.x + temp_object.object_rect.width;
      word_point.y = temp_object.object_rect.y;

      cvPutText(img_, file_name_ID, word_point, &font, word_color);
    }
  }


  if (1)//画速度指示线
  {
    float temp_angle = ego_veh_state_current.global_position.heading*pi/180;
    CvPoint temp_v_s;
    CvPoint temp_v_e;
    CvPoint temp_v4_s[4];
    CvPoint temp_v4_e[4];
    float temp_v_x = temp_object.motion_state_ab.v_x_post;//Kalman滤波输出绝对速度
    float temp_v_y = temp_object.motion_state_ab.v_y_post;
    CvPoint2D32f temp_point_f;//像素坐标系下中心点

    temp_point_f.x = center_point.x;
    temp_point_f.y = center_point.y;
    //绘制速度指示线是从中心点开始画的
    temp_v_s = center_point;
    //将目标全局速度转到车体坐标系
    temp_point_f.x = temp_point_f.x + (temp_v_x*cos(temp_angle) - temp_v_y*sin(temp_angle))/map_resolution/1.5;
    temp_point_f.y = temp_point_f.y - (temp_v_x*sin(temp_angle) + temp_v_y*cos(temp_angle))/map_resolution/1.5;
    temp_v_e = cvPointFrom32f(temp_point_f);

    if (0)
    {
      for (int m = 0; m < 4; ++m)
      {
        temp_point_f.x = (temp_object.shape.point4[m].x/map_resolution + img_->width/2);
        temp_point_f.y = (img_->height-1 - temp_object.shape.point4[m].y/map_resolution - map_offset_y);
        temp_v4_s[m] = cvPointFrom32f(temp_point_f);
        temp_v_x = temp_object.motion_state_ab.v_x_pt4[m];
        temp_v_y = temp_object.motion_state_ab.v_y_pt4[m];

        temp_point_f.x = temp_point_f.x + (temp_v_x*cos(temp_angle) - temp_v_y*sin(temp_angle))/map_resolution/1.5;
        temp_point_f.y = temp_point_f.y - (temp_v_x*sin(temp_angle) + temp_v_y*cos(temp_angle))/map_resolution/1.5;
        temp_v4_e[m] = cvPointFrom32f(temp_point_f);
      }
    }

    cvLine(img_, temp_v_e, temp_v_s, cvScalar(175,0, 175), 1,8,0);

    if(1)//目标周围显示ID号和速度
    {
      CvFont font;
      cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 0.8f, 0, 1, 8);

      char file_name_ID[50];
      memset(file_name_ID, 0, 50);
      float temp_v = sqrt(temp_v_x*temp_v_x + temp_v_y*temp_v_y);
      //预测部分轨迹拟合出来的速度
//      float predict_v = sqrt(pow(temp_object.predict_traj[0].v_x,2)+pow(temp_object.predict_traj[0].v_y,2));
      sprintf(file_name_ID, "%d %.2f",temp_object.target_ID,temp_v);
      CvScalar word_color = cvScalar( 0, 0, 255 );
      CvPoint word_point;
      word_point.x = center_point.x + temp_object.object_rect.width/2;
      word_point.y = center_point.y - temp_object.object_rect.height/2;
      cvPutText(img_, file_name_ID, word_point, &font, word_color);
    }
  }

  if (1)//画历史运动轨迹,需要将历史全局信息转到当前车体坐标系下
  {
    CvPoint temp_point_pre;
    for (int m=0; m<temp_object.history_info.size(); ++m){
      CvPoint temp_point4[4];
      CvPoint2D32f temp_point_f;
      //中心点全局坐标
      Point_d temp_point_g = temp_object.history_info[m].history_center_ab;
      //先转到当前车体坐标系下
      BaseFunction::point_global_to_local(temp_point_g.x, temp_point_g.y, ego_veh_state_current, &temp_point_f.x, &temp_point_f.y);
      //再转成像素坐标
      temp_point_f.x = (temp_point_f.x/map_resolution + img_->width/2);
      temp_point_f.y = (img_->height-1 - temp_point_f.y/map_resolution - map_offset_y);
      CvPoint temp_center = cvPointFrom32f(temp_point_f);

      CvPoint temp_point = temp_center;
      cvCircle(img_, temp_point, 1, cvScalar(20,225- m*7,100+ m*5),1,8,0);

      if (m > 0)
        cvLine(img_, temp_point_pre, temp_point, cvScalar(20,255- m*6,100+ m*5), 1,8,0);
      temp_point_pre = temp_point;
    }
  }

  if (0)//显示预测轨迹
  {
//    CvPoint temp_point_pre;
//    for (int i = 0; i < temp_object.predict_num; ++i)
//    {
//      Point_d predict_point = temp_object.predict_traj[i].point;
//      CvPoint2D32f temp_point_f = temp_object.center_point;
//      BaseFunction::point_global_to_local(predict_point.x, predict_point.y, ego_veh_state_current, &temp_point_f.x, &temp_point_f.y);
//      temp_point_f.x = (temp_point_f.x/map_resolution + img_->width/2);
//      temp_point_f.y = (img_->height-1 - temp_point_f.y/map_resolution - map_offset_y);
//      CvPoint temp_point = cvPointFrom32f(temp_point_f);
//      cvCircle(img_, temp_point, 1, cvScalar(255,125,0), 1, 8,0);
//      if(i > 0)
//      {
//        cvLine(img_, temp_point, temp_point_pre, cvScalar(255,0,255), 1, 8,0);
//      }
//      temp_point_pre = temp_point;
//    }
  }
  if (0)//显示匹配范围
  {
    CvPoint2D32f temp_point;
    temp_point.x = temp_object.center_point.x/map_resolution + img_->width/2;
    temp_point.y = img_->height-1 - temp_object.center_point.y/map_resolution - map_offset_y;
    CvPoint center_point = cvPointFrom32f(temp_point);
    if (temp_object.object_type < 2)
    {
      cvCircle(img_, center_point, temp_object.match_range.d_radius/map_resolution, cvScalar(0,255,0), 1);
    }
    CvPoint temp_point4[4];
    for (int m = 0; m < 4; ++m)
    {
      CvPoint2D32f temp_point;
      temp_point.x = temp_object.match_range.range_rect4[m].x/map_resolution + img_->width/2;
      temp_point.y = img_->height-1 - temp_object.match_range.range_rect4[m].y/map_resolution - map_offset_y;
      temp_point4[m] = cvPointFrom32f(temp_point);
    }
    CvPoint* ppt_test = temp_point4;

    int point_count = 4;
    cvPolyLine(img_, &ppt_test, &point_count, 1, 1, cvScalar(255,255,0), 1);
  }
}

void ShowResult::Show_moving_vector(cv::Mat& img,
                                    vector<MovingObject> moving_object_vector,
                                    const State_Vehicle& ego_veh_state_current)
{
  IplImage* img_ipl = new IplImage(img);
  for (int i = 0; i < moving_object_vector.size(); ++i) {
    MovingObject temp_object = moving_object_vector.at(i);
    ShowObject_moving(img_ipl, temp_object, ego_veh_state_current);
  }
}
