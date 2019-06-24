#include "StructMovingTargetDefine.h"
using namespace std;

//////////////////////////////// OGMProperty ////////////////////////////////
OGMProperty::OGMProperty():
    ogm_width_(0), ogm_height_(0), ogm_y_offset_(0), resolution_(0.0)
{

}
OGMProperty::OGMProperty(float ogm_width, float ogm_height, float ogm_y_offset,float resolution):
    ogm_width_(ogm_width),
    ogm_height_(ogm_height),
    ogm_y_offset_(ogm_y_offset),
    resolution_(resolution)
{

}

int OGMProperty::mapOffset() const
{
  return std::round(ogm_y_offset_ / resolution_);
}

cv::Size OGMProperty::mapSize() const
{
  return cv::Size(std::round(ogm_width_ / resolution_) + 1,
                  std::round(ogm_height_ / resolution_) + 1);
}

int OGMProperty::mapCellNum() const
{
  return mapSize().width * mapSize().height;
}

cv::Point OGMProperty::mapAnchor() const
{
  cv::Point res;
  res.x = mapSize().width/2;
  res.y = mapSize().width - 1 - mapOffset();
  return res;
}

////////////////////////////////////PolarProperty///////////////////////////////////////////////////
PolarProperty::PolarProperty(float radial_limit, float radial_resolution, float circular_limit, float circular_resolution):
  radial_limit_(radial_limit),radial_resolution_(radial_resolution),
  circular_limit_(circular_limit), circular_resolution_(circular_resolution)
{

}

int PolarProperty::radialCellsNum() const
{
  return std::round(radial_limit_/radial_resolution_);
}

int PolarProperty::circularCellsNum() const
{
  return std::round(circular_limit_ / circular_resolution_);
}
int PolarProperty::polarCellsNum() const
{
  return radialCellsNum() * circularCellsNum();
}

/////////////////////////////////// sensors_fusion /////////////////////////////////////////////////
namespace sensors_fusion {
/////////////////////////////////// EgoVehicleState /////////////////////////////////////////////////
EgoVehicleState::EgoVehicleState():
speed_(0), longitude_(0.0), latitude_(0.0), altitude_(0.0), heading_(0.0), pitch_(0.0), roll_(0.0)
{

}

EgoVehicleState::EgoVehicleState(const State_Vehicle& st):
    speed_(st.fForwardVel),
    longitude_(st.global_position.dLng), latitude_(st.global_position.dLat),
    altitude_(st.global_position.dAld), heading_(st.global_position.heading),
    pitch_(st.global_position.pitch), roll_(st.global_position.roll)
{

}

EgoVehicleState EgoVehicleState::EgoVehicle(const cv::Scalar& vehicleShape)
{
  auto egoVehicleState =  EgoVehicleState();
  egoVehicleState.vehicle_shape_ = vehicleShape;
  return egoVehicleState;
}

EgoVehicleState::operator State_Vehicle() const
{
  State_Vehicle res;
  res.fForwardVel = speed_;
  res.global_position.dLng = longitude_; res.global_position.dLat = latitude_;res.global_position.dAld = altitude_;
  res.global_position.heading = heading_; res.global_position.pitch = pitch_; res.global_position.roll = roll_;
  return res;
}

}



////////////////////////////////////BaseFunction///////////////////////////////////////////////////
float BaseFunction::Angle_atan(float d_x_, float d_y_)
{
  float temp_angle = 0;
  if (d_x_ == 0)
  {
    if (d_y_ > 0 )
      temp_angle = 90;
    if (d_y_ < 0 )
      temp_angle = -90;
  }
  else
  {
    float d_x_y = d_y_ / d_x_;
    temp_angle = atan(d_x_y);
    temp_angle = temp_angle * 180/pi;
  }
  return temp_angle;
}

float BaseFunction::Angle_atan2(float d_x_, float d_y_)//顺时针角度
{
  float temp_angle = (float)atan2(d_y_, d_x_);
  temp_angle = -temp_angle;
  if (temp_angle < 0)
    temp_angle = temp_angle + 2*pi;

  temp_angle = temp_angle*180/pi;
  if (temp_angle > 360)
    temp_angle = 359.999;

  return temp_angle;
}

float BaseFunction::Angle_line2(CvPoint2D32f line_1_, CvPoint2D32f line_2_)
{
  float line1_length = sqrt(line_1_.x*line_1_.x + line_1_.y*line_1_.y);
  float line2_length = sqrt(line_2_.x*line_2_.x + line_2_.y*line_2_.y);
  float line_product = (line_1_.x * line_2_.x + line_1_.y * line_2_.y);
  line_product = line_product / line1_length / line2_length;
  if (line_product>1)
    line_product = 1;

  if (line_product<-1)
    line_product = -1;

  float delta_side_angle0 = acos(line_product)*180/pi;

  return delta_side_angle0;
}

CvPoint2D32f BaseFunction::Point_line2(Line_s line1_, Line_s line2_)
{
  CvPoint2D32f temp_point;
  if ( fabs(line1_.a) < fabs(line1_.b) )
  {
    temp_point.x = line1_.c * line1_.a + line2_.c * line2_.a;
    temp_point.y = (line1_.c - temp_point.x * line1_.a) / line1_.b;
  }
  else
  {
    temp_point.y = line1_.c * line1_.b + line2_.c * line2_.b;
    temp_point.x = (line1_.c - temp_point.y * line1_.b) / line1_.a;
  }
  return temp_point;

}
float BaseFunction::Angle_FitLinePoints(vector<CvPoint2D32f> seed_point_vector_, int point_num_)
{
  CvPoint2D32f* seed_points = (CvPoint2D32f*)malloc( point_num_ * sizeof(seed_points[0]));
  for (int i=0; i<seed_point_vector_.size(); i++)
  {
    CvPoint2D32f seed_point = seed_point_vector_.at(i);
    seed_points[i] = seed_point;//cvPointFrom32f(seed_point);
  }
  float fit_line_param[4];
  memset(fit_line_param, 0 , 4*sizeof(float));
  CvMat pointMat = cvMat( 1, point_num_, CV_32FC2, seed_points );
  cvFitLine( &pointMat, CV_DIST_L1, 1, 0.01, 0.01, fit_line_param );

  float fit_line_angle = atan2(fit_line_param[1] , fit_line_param[0]);

  return fit_line_angle;
}
int BaseFunction::value_in_threshod_int(int value_, int threshod_pre_, int threshod_up_)
{
  int value_return = value_;
  int threshod_length = threshod_up_ - threshod_pre_ + 1;

  if (value_ < threshod_pre_)
  {
    value_return = value_ + threshod_length;
  }
  if (value_ > threshod_up_)
  {
    value_return = value_ - threshod_length;
  }

  return value_return;
}

bool BaseFunction::isPointInRect(int x, int y,cv::Point A, cv::Point B, cv::Point C, cv::Point D)
{
  int areaAB = abs((A.x - x)*(B.y - y) - (A.y - y)*(B.x - x));
  int areaBC = abs((B.x - x)*(C.y - y) - (B.y - y)*(C.x - x));
  int areaCD = abs((C.x - x)*(D.y - y) - (C.y - y)*(D.x - x));
  int areaDA = abs((D.x - x)*(A.y - y) - (D.y - y)*(A.x - x));
  int areaABCD = abs((A.x-B.x)*(C.y-B.y)-(A.y-B.y)*(C.x-B.x));
  if( (areaAB + areaBC + areaCD + areaDA) == 2*areaABCD ) {
    return true;
  }
  return false;
}

bool BaseFunction::isPointInPolygon(int x, int y, cv::Point vertex[], int nvert)
{
  int i, j;;
  bool c = false;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((vertex[i].y > y) != (vertex[j].y > y)) &&
        (x < (vertex[j].x - vertex[i].x) * (y - vertex[i].y) / (vertex[j].y - vertex[i].y) + vertex[i].x) )
      c = !c;
  }
  return c;
}

bool BaseFunction::isPointInPolygon(int x, int y, std::vector<cv::Point> vertex)
{
  int nvert = vertex.size();
  int i, j;;
  bool c = false;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((vertex[i].y > y) != (vertex[j].y > y)) &&
        (x < (vertex[j].x - vertex[i].x) * (y - vertex[i].y) / (vertex[j].y - vertex[i].y) + vertex[i].x) )
      c = !c;
  }
  return c;
}

void BaseFunction::point_local_to_global(float x_, float y_, State_Vehicle ego_state_, double* lng_x_, double* lat_y_)
{
  double temp_angle = ego_state_.global_position.heading*pi/180;
//  double temp_lng = x_*cos(-temp_angle) - y_*sin(-temp_angle) + ego_state_.global_position.dLng;
//  double temp_lat = x_*sin(-temp_angle) + y_*cos(-temp_angle) + ego_state_.global_position.dLat;

  double temp_lng = x_*cos(temp_angle) + y_*sin(temp_angle) + ego_state_.global_position.dLng;
  double temp_lat = -x_*sin(temp_angle) + y_*cos(temp_angle) + ego_state_.global_position.dLat;
  *lng_x_ = temp_lng;
  *lat_y_ = temp_lat;
}

void BaseFunction::point_global_to_local(double lng_x_, double lat_y_ ,State_Vehicle ego_state_, float* x_, float* y_)
{
  double temp_angle = ego_state_.global_position.heading*pi/180;

  double temp_lng = lng_x_ - ego_state_.global_position.dLng;
  double temp_lat = lat_y_ - ego_state_.global_position.dLat;
  double temp_x = temp_lng*cos(temp_angle) - temp_lat*sin(temp_angle);
  double temp_y = temp_lng*sin(temp_angle) + temp_lat*cos(temp_angle);
  *x_ = temp_x;
  *y_ = temp_y;
}

//给定车体坐标系下点坐标(米制单位),计算对应栅格地图像素坐标
void BaseFunction::point_veh_to_img(IplImage* img_, CvPoint* point_, CvPoint2D32f point_f_, float resolution_, int offest_y_)
{
  CvPoint2D32f temp_point_f;
  temp_point_f.x = (point_f_.x/resolution_ + img_->width/2);
  temp_point_f.y = (img_->height-1 - point_f_.y/resolution_ - offest_y_);
  *point_ = cvPointFrom32f(temp_point_f);
}

void BaseFunction::point_img_to_veh(IplImage* img_, CvPoint point_, CvPoint2D32f* point_f_, float resolution_, int offest_y_)
{
  CvPoint2D32f temp_point_f;
  temp_point_f.x = (point_.x - img_->width/2)*resolution_;
  temp_point_f.y = (img_->height-1 - point_.y - offest_y_)*resolution_;
  *point_f_ = temp_point_f;
}

void BaseFunction::Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0, double *e, double *n)
{
  double WGS84_ECCENTRICITY = (double)0.0818192;                        // e=0.0818192
  double WGS84_EQUATORIAL_RADIUS = (double)6378.137;                    // a=6378.137
  double k0 = (double)0.9996;

  int Zone = (int)(longitude/6) + 1;
  int lonBase = Zone*6 - 3;

  double   vPhi = (double)(1 / sqrt(1-pow(WGS84_ECCENTRICITY * sin(latitude*CV_PI/180.0),2)));
  double  A    = (double)(( longitude - lonBase )*CV_PI/180.0 * cos(latitude*CV_PI/180.0));
  double  sPhi = (double)((1 - pow(WGS84_ECCENTRICITY,2)/4.0 - 3*pow(WGS84_ECCENTRICITY,4)/64.0 - 5*pow(WGS84_ECCENTRICITY,6)/256.0) * latitude*CV_PI/180.0
      - (3*pow(WGS84_ECCENTRICITY,2)/8.0 + 3*pow(WGS84_ECCENTRICITY,4)/32.0 + 45*pow(WGS84_ECCENTRICITY,6)/1024.0) * sin(2*latitude*CV_PI/180.0)
      + (15*pow(WGS84_ECCENTRICITY,4)/256.0 + 45*pow(WGS84_ECCENTRICITY,6)/256.0)* sin(4*latitude*CV_PI/180.0)
      - (35*pow(WGS84_ECCENTRICITY,6)/3072.0) * sin(6*latitude*CV_PI/180.0));
  double  T   = (double)(pow(tan(latitude*CV_PI/180.0),2));
  double  C   = (double)((pow(WGS84_ECCENTRICITY,2)/(1 - pow(WGS84_ECCENTRICITY,2))) * pow(cos(latitude*CV_PI/180.0),2));

  *e = (double)((k0*WGS84_EQUATORIAL_RADIUS*vPhi*(A + (1 - T + C)*pow(A,3)/6.0
      + (5 - 18*T + pow(T,2))*pow(A,5)/120.0))*1000 - e0);
  *n = (double)((k0*WGS84_EQUATORIAL_RADIUS*(sPhi + vPhi * tan(latitude*CV_PI/180.0)*(pow(A,2)/2
      + (5 - T + 9*C + 4*C*C)*pow(A,4)/24.0 + (61 - 58*T + T*T)*pow(A,6)/720.0)))*1000 - n0);
}

void BaseFunction::InitVirtualLine(std::vector<Line_int>& line_int)
{
  for (size_t i = 0; i < line_int.size(); ++i) {
    line_int[i].x1 = 0;
    line_int[i].y1 = 0;
    line_int[i].x2 = 0;
    line_int[i].y2 = 0;
    line_int[i].line_length= 0;
    line_int[i].angle= 0;
    line_int[i].type = 0;
    line_int[i].left_invalid = 0;
    line_int[i].right_invalid = 0;
  }
}

void BaseFunction::InitVirtualLine(Line_int* line_int,int line_num)
{
  for(int i = 0; i < line_num; ++i){
    line_int[i].x1 = 0;
    line_int[i].y1 = 0;
    line_int[i].x2 = 0;
    line_int[i].y2 = 0;
    line_int[i].line_length= 0;
    line_int[i].angle= 0;
    line_int[i].type = 0;
    line_int[i].left_invalid = 0;
    line_int[i].right_invalid = 0;
  }
}

void BaseFunction::InitOGM( int ogm_cell_size_, OGM_Cell* rigid_ogm_ )
{
  for (int i = 0; i<ogm_cell_size_; i++) {
    rigid_ogm_[i].intensity = 0;
    rigid_ogm_[i].type = CellProperty::Unknown;
    rigid_ogm_[i].min_z = 1000;
    rigid_ogm_[i].max_z = -1000;
    rigid_ogm_[i].average_z = 0;
    rigid_ogm_[i].delta_z = 0;

    rigid_ogm_[i].layers_num =0;
    rigid_ogm_[i].points_num = 0;
    rigid_ogm_[i].endpoint_index = -1;
    rigid_ogm_[i].startpoint_index = 1000000;
    rigid_ogm_[i].startlaser_index = 32;
    rigid_ogm_[i].endlaser_index = -1;

    rigid_ogm_[i].ground_z = 0;
    rigid_ogm_[i].cloud_index.clear();
//    vector<int>().swap(rigid_ogm_[i].cloud_index);
  }
}

void BaseFunction::InitOGM(std::vector<OGM_Cell>& rigid_ogm_)
{
  for (int i = 0; i<rigid_ogm_.size(); i++)
  {
    rigid_ogm_[i].intensity = 0;
    rigid_ogm_[i].type = CellProperty::Unknown;
    rigid_ogm_[i].min_z = 1000;
    rigid_ogm_[i].max_z = -1000;
    rigid_ogm_[i].average_z = 0;
    rigid_ogm_[i].delta_z = 0;

    rigid_ogm_[i].layers_num =0;
    rigid_ogm_[i].points_num = 0;
    rigid_ogm_[i].endpoint_index = -1;
    rigid_ogm_[i].startpoint_index = 1000000;
    rigid_ogm_[i].startlaser_index = 32;
    rigid_ogm_[i].endlaser_index = -1;

    rigid_ogm_[i].ground_z = 0;
    rigid_ogm_[i].cloud_index.clear();
  }
}

void BaseFunction::InitPolarOGM(vector<Polar_Cell>& polar_ogm)
{
  for (int i = 0; i<polar_ogm.size(); i++) {
    polar_ogm[i].intensity = 0;
    polar_ogm[i].type = CellProperty::Unknown;
    polar_ogm[i].min_z = 1000;
    polar_ogm[i].max_z = -1000;
    polar_ogm[i].average_z = 0;
    polar_ogm[i].delta_z = 0;
    polar_ogm[i].points_num = 0;
    polar_ogm[i].ground_z = 0;
    polar_ogm[i].grad_m = 0;
    polar_ogm[i].grad_b = 0;
    polar_ogm[i].up_link = false;
    polar_ogm[i].down_link = false;
    polar_ogm[i].left_link = false;
    polar_ogm[i].right_link = false;
  }
}

void BaseFunction::InitPolarOGM( int polarogm_cell_size_, Polar_Cell* polar_ogm_ )
{
  for (int i = 0; i<polarogm_cell_size_; i++) {
    polar_ogm_[i].intensity = 0;
    polar_ogm_[i].type = CellProperty::Unknown;
    polar_ogm_[i].min_z = 1000;
    polar_ogm_[i].max_z = -1000;
    polar_ogm_[i].average_z = 0;
    polar_ogm_[i].delta_z = 0;

    polar_ogm_[i].points_num = 0;
    polar_ogm_[i].ground_z = 0;
    polar_ogm_[i].grad_m = 0;
    polar_ogm_[i].grad_b = 0;
    polar_ogm_[i].up_link = false;
    polar_ogm_[i].down_link = false;
    polar_ogm_[i].left_link = false;
    polar_ogm_[i].right_link = false;
  }
}

void BaseFunction::InitRegion( int region_cell_size_, Region_Cell* region_ )
{
  for (int i = 0; i<region_cell_size_; i++)
  {
    region_[i].object_num = 0;
    region_[i].object_index_end = -1;
    region_[i].object_index_start = MAX_NUM_TARGET;

    region_[i].layers_num = 0;
    region_[i].startlaser_index = 0;
    region_[i].endlaser_index = -1;
    region_[i].endpoint_index = -1;
    region_[i].startpoint_index = 1000000;

    region_[i].ground_endz = 1000;
    region_[i].ground_startz = 0;
    region_[i].disxy_end = -1;
    region_[i].disxy_start = 50;
    region_[i].ground_type = 0;
  }
}

void BaseFunction::MovingTarget2Send(MovingObject target_send_, Target_output2* target_output_)
{
  target_output_->ID_number = target_send_.target_ID;
  //中心点全局位置
  target_output_->center_point.x = target_send_.center_pt_meas_ab.x;
  target_output_->center_point.y = target_send_.center_pt_meas_ab.y;

  target_output_->line_num = 4;
  for (int i = 0; i < 4; ++i) {//目标矩形框四个点
    target_output_->line_point[i].x = target_send_.contour_rect_ab.point4[i].x - target_send_.center_pt_meas_ab.x;
    target_output_->line_point[i].y = target_send_.contour_rect_ab.point4[i].y - target_send_.center_pt_meas_ab.y;
  }

  target_output_->object_high = target_send_.shape.object_height;
  target_output_->object_type = target_send_.object_type;

  target_output_->is_updated = target_send_.is_new_add?true:target_send_.has_match;//第一帧认为更新

  target_output_->tracked_times = target_send_.track_state.tracked_times;
  target_output_->dangerous_level = target_send_.dangerous_level;

  //预测信息
  target_output_->predict_info = target_send_.predict_info;

  //更改发送策略
  //1)第一次发送,发送所有历史轨迹
  int history_num = 0;
  if(1 == target_send_.send_times) {
    history_num = std::min((int)target_send_.history_info.size(),20);//最多发送历史20帧
  }
  else if(target_send_.send_times > 1) {
    history_num = 2;//只发送当前帧和上一帧
  }
  target_output_->history_num = history_num;

  //倒序,从最新开始
  for(auto iter = target_send_.history_info.rbegin();iter!=target_send_.history_info.rbegin()+history_num;++iter){
    int i = iter - target_send_.history_info.rbegin();
    target_output_->history_traj[i].history_time = iter->time_stamp;//时间戳
    target_output_->history_traj[i].history_center = iter->history_center_ab;//目标中心点
    target_output_->history_traj[i].history_rect = iter->history_rect_ab;//全局目标矩形框
  }//end for
}

void BaseFunction::MovingTarget2Send(const MovingObject& moving_target, TargetOutput& target_send)
{
  //目标ID
  target_send.target_ID = moving_target.target_ID;
  //中心点全局位置
  target_send.center_pt_ab = moving_target.center_pt_meas_ab;
  //中心点当前车体下位置
  target_send.center_pt = moving_target.center_pt;
  //目标矩形框坐标
  for (int i = 0; i < 4; ++i)//目标矩形框四个点,相对目标中心点坐标系
  {
    target_send.line_point[i].x = moving_target.contour_rect_ab.point4[i].x - moving_target.center_pt_meas_ab.x;
    target_send.line_point[i].y = moving_target.contour_rect_ab.point4[i].y - moving_target.center_pt_meas_ab.y;
  }

  //目标预测信息
  target_send.predict_traj = moving_target.predict_info;

  //目标历史信息发送策略
  //1)第一次发送,发送所有历史轨迹
  int history_num = 0;
  if(1 == moving_target.send_times){
    history_num = std::min((int)moving_target.history_info.size(),20);//最多发送历史20帧
  }
  else if(moving_target.send_times > 1){
    history_num = 2;//只发送当前帧和上一帧
  }

  //倒序,从最新开始
  for(auto iter = moving_target.history_info.rbegin();iter!=moving_target.history_info.rbegin()+history_num;++iter){
    target_send.history_info.push_back(*iter);
  }//end for

}

std::vector<Line_int> BaseFunction::SearchOccupiedLine(const cv::Mat& img_binary,const cv::Point& base_point, float distance)
{
  const float angle_resolution = 1.0;//1度
  const float distance_resolution = 0.2;//0.2m
  const int numChannel = floor(360.0/angle_resolution);
  std::vector<Line_int> virtual_line(numChannel);

  int search_distance = floor(distance/distance_resolution);
  for (int i=0; i<virtual_line.size(); i++){//遍历每个方位射线
      //射线初始化
    virtual_line[i].x1 = base_point.x;
    virtual_line[i].y1 = base_point.y;
    virtual_line[i].x2 = base_point.x;
    virtual_line[i].y2 = base_point.y;
    virtual_line[i].angle = i * angle_resolution;//resolution_ = 1.0 deg
    virtual_line[i].type = 0;//未找到状态
    virtual_line[i].line_length = 200.0;//一个较大的数
    float temp_angle = virtual_line[i].angle * pi / 180;

    for(int j = 0; j < search_distance; ++j){//一个方位上径向查找
      CvPoint2D32f temp_point_f;
      temp_point_f.x = base_point.x + j*cos(temp_angle);
      temp_point_f.y = base_point.y + j*sin(temp_angle);
      CvPoint temp_point = cvPointFrom32f(temp_point_f);

      if (temp_point.x >= 0 &&
          temp_point.x <= (img_binary.cols-1) &&
          temp_point.y >= 0&&
          temp_point.y <= (img_binary.rows -1)){//在图像范围内
         if(img_binary.at<uchar>(temp_point.y,temp_point.x) > 0){
           virtual_line[i].type = 1;//找到占据点
           virtual_line[i].x2 = temp_point.x;
           virtual_line[i].y2 = temp_point.y;
         }
      }
      else{
        break;
      }

      if(virtual_line[i].type == 1)//第一次找到即可
        break;
    }
    if(virtual_line[i].type == 1){//找到目标,才更新射线长度
      //该方位射线其他属性
      float temp_x = virtual_line[i].x1 - virtual_line[i].x2;
      float temp_y = virtual_line[i].y1 - virtual_line[i].y2;
      virtual_line[i].line_length = sqrt(temp_x*temp_x + temp_y*temp_y);
    }
  }
  return virtual_line;
}
