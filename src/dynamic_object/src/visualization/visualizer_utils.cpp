#include <visualization/visualizer_utils.h>

using std::vector;
using std::cout;
using std::endl;

namespace dynamic_object_tracking {

VisualizerUtils* VisualizerUtils::uniqueInstance = new VisualizerUtils();

VisualizerUtils* VisualizerUtils::getInstance()
{
  return uniqueInstance;
}

void VisualizerUtils::Init(float ogm_width,float ogm_height,float ogm_offset_y,float ogm_resolution,
                          int map_width,int map_height,int map_offset_y)
{
  this->ogm_width_ = ogm_width;
  this->ogm_height_ = ogm_height;
  this->ogm_offset_y_ = ogm_offset_y;
  this->ogm_resolution_ = ogm_resolution;

  this->map_width_ = map_width;
  this->map_height_ = map_height;
  this->map_offset_y_ = map_offset_y;
}

void VisualizerUtils::Init(const OGMProperty& ogm_property)
{
  this->ogm_width_ = ogm_property.ogm_width_;
  this->ogm_height_ = ogm_property.ogm_height_;
  this->ogm_offset_y_ = ogm_property.ogm_y_offset_;
  this->ogm_resolution_ = ogm_property.resolution_;

  this->map_width_ = ogm_property.mapSize().width;
  this->map_height_ = ogm_property.mapSize().height;
  this->map_offset_y_ = ogm_property.mapOffset();
}

bool VisualizerUtils::get_image_coord(float xv,float yv,int& row,int& col)
{
  //计算在图像位置
  //栅格地图预先设置好
  col = (int)((xv + ogm_width_/2) / ogm_resolution_);
  row = map_height_ - 1 - (int)((yv + ogm_offset_y_) / ogm_resolution_);
  if(col < 0||col>=map_width_||row<0||row>=map_height_)
    return false;
  return true;//在图像范围内,有效
}

void VisualizerUtils::DrawEgoVehicle(cv::Mat& img)
{
  CvPoint ego_vehicle = cvPoint( img.cols/2, (int)(img.rows - map_offset_y_) );
  int vehicle_width = EGO_VEHICLE_WIDTH/ ogm_resolution_;
  int vehicle_height = EGO_VEHICLE_LENGTH/ ogm_resolution_;
  CvPoint ego_vehicle_lt = cvPoint(ego_vehicle.x - vehicle_width/2, ego_vehicle.y - vehicle_height/2);
  CvPoint ego_vehicle_rb = cvPoint(ego_vehicle.x + vehicle_width/2, ego_vehicle.y + vehicle_height/2);
  cv::rectangle(img,ego_vehicle_lt,ego_vehicle_rb,cvScalar(0,0,0),-1,8,0);

  int tire_width = 1/ogm_resolution_;
  int tire_height = 2/ogm_resolution_;

  CvPoint ego_vehicle_ltb = cvPoint(ego_vehicle_lt.x + tire_width/2,ego_vehicle_lt.y + tire_height/2);
  cv::rectangle(img,ego_vehicle_lt,ego_vehicle_ltb,cvScalar(0,255,255),-1,8,0);

  CvPoint ego_vehicle_rtt = cvPoint(ego_vehicle.x + vehicle_width/2,ego_vehicle.y - vehicle_height/2);
  CvPoint ego_vehicle_rtb = cvPoint(ego_vehicle_rtt.x - tire_width/2,ego_vehicle_rtt.y + tire_height/2);
  cv::rectangle(img,ego_vehicle_rtt,ego_vehicle_rtb,cvScalar(0,255,255),-1,8,0);
  CvPoint ego_vehicle_rbt = cvPoint(ego_vehicle_rb.x - tire_width/2,ego_vehicle_rb.y - tire_height/2);
  cv::rectangle(img,ego_vehicle_rb,ego_vehicle_rbt,cvScalar(0,255,255),-1,8,0);

  CvPoint ego_vehicle_lbb = cvPoint(ego_vehicle.x - vehicle_width/2,ego_vehicle.y + vehicle_height/2);
  CvPoint ego_vehicle_lbt = cvPoint(ego_vehicle_lbb.x + tire_width/2,ego_vehicle_lbb.y - tire_height/2);
  cv::rectangle(img,ego_vehicle_lbb,ego_vehicle_lbt,cvScalar(0,255,255),-1,8,0);

  cv::Point pt[3];
  pt[0] = cvPoint(ego_vehicle.x,ego_vehicle.y - 1.2/ogm_resolution_);
  pt[1] = cvPoint(ego_vehicle.x - 0.6/ogm_resolution_, ego_vehicle.y + 0.8/ogm_resolution_);
  pt[2] = cvPoint(ego_vehicle.x + 0.6/ogm_resolution_, ego_vehicle.y + 0.8/ogm_resolution_);

  cv::fillConvexPoly(img,pt,3,cv::Scalar(0,255,0),8,0);

  int dis_10 = boost::math::round(10/ogm_resolution_);
  int dis10_width_num = 3;
  int dis10_height_num = 3;
  for (int i=0; i<(dis10_height_num); i++)
  {
    CvPoint line_point0;
    CvPoint line_point1;
    line_point0.x = 0;
    line_point0.y = ego_vehicle.y + i*20/ogm_resolution_;
    line_point1.x = img.cols;
    line_point1.y = ego_vehicle.y + i*20/ogm_resolution_;
    cv::line(img, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
    if(1)
    {
      CvFont font;
      cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);
      char file_name_ID[50];
      sprintf(file_name_ID, "%dm ", -i*20);
      cv::Scalar word_color(125,125,125);
      cv::Point word_point;
      word_point.x = line_point0.x +2;
      word_point.y = line_point0.y;
      std::string text(file_name_ID);
//      cvPutText(img, text, word_point, &font, word_color);
      cv::putText(img,file_name_ID,word_point,cv::FONT_HERSHEY_SIMPLEX,0.5,word_color,1);
    }
    if (i>0)
    {
      line_point0.y = ego_vehicle.y - i*20/ogm_resolution_;
      line_point1.y = ego_vehicle.y - i*20/ogm_resolution_;
      cv::line(img, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
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
//        cvPutText(img_, file_name_ID, word_point, &font, word_color);
        cv::putText(img,file_name_ID,word_point,cv::FONT_HERSHEY_COMPLEX,0.5,word_color,1);
      }
    }
  }
  for (int i=0; i < dis10_width_num; i++)
  {
    CvPoint line_point0;
    CvPoint line_point1;

    line_point0.x = ego_vehicle.x + i*20/ogm_resolution_ + 10/ogm_resolution_;
    line_point0.y = 0;
    line_point1.x = ego_vehicle.x + i*20/ogm_resolution_ + 10/ogm_resolution_;
    line_point1.y = img.rows;
    cv::line(img, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);

    line_point0.x = ego_vehicle.x - i*20/ogm_resolution_ - 10/ogm_resolution_;
    line_point1.x = ego_vehicle.x - i*20/ogm_resolution_ - 10/ogm_resolution_;
    cv::line(img, line_point0, line_point1, cvScalar(125,125,125), 1, 8, 0);
    if (i==0)
    {
      line_point0.x = ego_vehicle.x;
      line_point0.y = ego_vehicle.y + 10/ogm_resolution_;
      line_point1.x = ego_vehicle.x;
      line_point1.y = ego_vehicle.y - 10/ogm_resolution_;
      cv::line(img, line_point0, line_point1, cvScalar(155,155,155), 1, 8, 0);
    }
  }
}

void VisualizerUtils::DrawEgoPoints(cv::Mat& img, float x, float y, cv::Scalar color)
{
  assert(img.cols == this->map_width_);
  assert(img.rows == this->map_height_);
  int row,col;
  if(!get_image_coord(x,y,row,col))
    return;
  cv::circle(img,cv::Point(col,row),2,color,-1);
}

cv::Mat VisualizerUtils::DrawPointCloudOGM(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, bool draw_vehicle,cv::Scalar color)
{
  cv::Mat img_res = cv::Mat::zeros(map_height_,map_width_,CV_8UC3);
  if(draw_vehicle)
    this->DrawEgoVehicle(img_res);
  for(size_t i = 0;i<cloud_in->size();++i)
  {
    //获得每个点的属性
    float x = cloud_in->points[i].x;
    float y = cloud_in->points[i].y;
    float z = cloud_in->points[i].z;
    //计算在图像位置
    int row,col;
    if(!get_image_coord(x,y,row,col))
      continue;
    img_res.at<cv::Vec3b>(row,col) = cv::Vec3b(color(0),color(1),color(2));//BGR color
  }
  return img_res;
}

void VisualizerUtils::ShowIplImage(std::string window_name,const IplImage* img_src,bool copyData, bool draw_vehicle)
{
  if(img_src == NULL)
    return;
  assert(img_src->width == this->map_width_);
  assert(img_src->height == this->map_height_);
  cv::Mat img_tmp = cv::cvarrToMat(img_src,copyData);
  if(draw_vehicle)
    this->DrawEgoVehicle(img_tmp);
  cv::namedWindow(window_name.c_str(),CV_WINDOW_NORMAL);
  cv::imshow(window_name.c_str(),img_tmp);
  cv::waitKey(2);
}


void VisualizerUtils::ShowText(cv::Mat& img, long long frame_counter,
                               int target_num, float cost_time, float ego_speed)
{
  std::ostringstream sstream;
  sstream<<"frame: "<<frame_counter; //显示帧数
  CvPoint temp_point = cvPoint(5,35);
  cv::putText(img, sstream.str(), temp_point, CV_FONT_HERSHEY_PLAIN, 1.3, cvScalar(255,255,0),2);

  char name_framecounter[50];
  memset(name_framecounter, 0, 50);
  sprintf(name_framecounter,"egoVel: %.1fm/s", ego_veh_state_current.fForwardVel);//显示本车车速
  temp_point = cvPoint(5,65);
  cv::putText(img, name_framecounter, temp_point, CV_FONT_HERSHEY_PLAIN, 1.3, cvScalar(255,255,0), 2);

  sstream.str("");
  sstream<<"target: "<<target_num;//显示动态目标数
  temp_point = cvPoint(5,95);
  cv::putText(img, sstream.str().c_str(), temp_point, CV_FONT_HERSHEY_PLAIN, 1.3, cvScalar(255,255,0), 2);

  sprintf(name_framecounter,"time: %.3fs", cost_time);//显示处理时间
  temp_point = cvPoint(5,125);
  cv::putText(img, name_framecounter, temp_point, CV_FONT_HERSHEY_PLAIN, 1.3, cvScalar(255,255,0), 2);
}

void VisualizerUtils::ShowMovingObjectVector(IplImage* img_, vector<MovingObject>* moving_object_vector)
{
  float map_resolution = this->ogm_resolution_;
  int map_offset_y = this->map_offset_y_;
  cout<<"map_resolutionnnnnnnnnnn "<<map_resolution<<" "<<map_offset_y<<" "<<moving_object_vector->size()<<endl;

  for (int i = 0; i < moving_object_vector->size(); i++){
    MovingObject temp_object = moving_object_vector->at(i);
    CvPoint contour_point4[4];
      CvPoint center_point;
      //得到目标中心点像素坐标
      center_point.x = (int)(temp_object.center_pt.x/map_resolution + img_->width/2);
      center_point.y = (int)(img_->height-1 - temp_object.center_pt.y/map_resolution - img_->height/2);
      cout<<"center_point======= "<<center_point.x<<" "<<center_point.y<<endl;
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
        cout<<"point======= "<<contour_point4[m].x<<" "<<contour_point4[m].y<<endl;
      }
      center_point.x = (contour_point4[0].x + contour_point4[2].x)/2;
      center_point.y = (contour_point4[0].y + contour_point4[2].y)/2;


      int point_count = 4;
      if (1)//绘制目标矩形框
      {
        cout<<"enter in cvPolyLine........"<<endl;
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

  if(moving_object_vector->size() > 0){
    cvNamedWindow("temp",CV_WINDOW_NORMAL);
    cvShowImage("temp",img_);
    cvWaitKey(5);
  }

}

} /* namespace dynamic_object_tracking */
