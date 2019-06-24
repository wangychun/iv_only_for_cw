#include "StructMovingTargetDefine.h"
#include <visualization/visualizer_utils.h>
#include "object_matching.h"
using namespace cv;

ObjectMatching::ObjectMatching():
is_show_result_(true),
candidate_object_num(0),
moving_object_num(0),
map_width(0),
map_height(0),
map_offset_y(0),
map_resolution(0.0),
virtual_line_num(360),
virtual_line_resolution(1.0),
match_analysis_map_(cv::Mat())
{
  init_matchstate_.is_match = false;
  init_matchstate_.match_index = -1;
  init_matchstate_.similar_num = 0;
  init_matchstate_.similar_index_vector.clear();
  init_matchstate_.max_similar_num = 0;
  init_matchstate_.max_similar_index_vector.clear();
}

ObjectMatching::~ObjectMatching()
{
}

void ObjectMatching::Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
        const int& virtual_line_num_, const float& virtual_line_resolution_,bool is_show_result)
{
  map_width = boost::math::round(ogm_width_ / ogm_resolution_);
  map_height = boost::math::round(ogm_height_ / ogm_resolution_);
  map_resolution = ogm_resolution_;
  map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);

  virtual_line_num = virtual_line_num_;
  virtual_line_resolution = virtual_line_resolution_;

  match_analysis_map_ = Mat::zeros(map_height,map_width + 200,CV_8UC3);
  this->is_show_result_ = is_show_result;
//  //绘制一条分界线
//  cv::line(match_analysis_map_,cv::Point(map_width-1,0),cv::Point(map_width - 1, map_height - 1),cv::Scalar(122,122,122),2);
}

void ObjectMatching::Init(int map_width, int map_height, int map_offest_y,float map_resolution,
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


//判断当前检测目标点是否落在矩形区域内
bool ObjectMatching::Point_in_region(CvPoint2D32f point_, CvPoint2D32f* region_points_)
{
  float temp_dot4[4];
  for (int m = 0; m < 4; ++m)
  {
    int m_up = BaseFunction::value_in_threshod_int(m+1, 0 , 3);
    temp_dot4[m] = (region_points_[m_up].x - region_points_[m].x)*(point_.y - region_points_[m].y)
                          - (region_points_[m_up].y - region_points_[m].y)*(point_.x - region_points_[m].x);
  }
  if( (temp_dot4[0] > 0 && temp_dot4[1] > 0 && temp_dot4[2] > 0 && temp_dot4[3] > 0)
      || (temp_dot4[0] < 0 && temp_dot4[1] < 0 && temp_dot4[2] < 0 && temp_dot4[3] < 0))
  {
    return true;
  }
  return false;
}

//匹配当前检测目标和已经跟踪目标矩形框四个对应顶点
int ObjectMatching::Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_)
{
  if (moving_object_.object_type <2 || candidate_object_.object_type<2)
    return 0;

  CvPoint2D32f line4_cd[4]; // 01 12
  CvPoint2D32f line4_mt[4]; // 01 12 23 30
  if (method_ == 1)//绝对坐标分析
  {
    for (int m=0; m<4; m++)//
    {
      int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
      line4_cd[m].x = candidate_object_.shape.point4[m_up].x - candidate_object_.shape.point4[m].x;
      line4_cd[m].y = candidate_object_.shape.point4[m_up].y - candidate_object_.shape.point4[m].y;
      line4_mt[m].x = moving_object_.contour_rect_pre.point4[m_up].x - moving_object_.contour_rect_pre.point4[m].x;
      line4_mt[m].y = moving_object_.contour_rect_pre.point4[m_up].y - moving_object_.contour_rect_pre.point4[m].y;
    }
  }

  int match_index0_cd = candidate_object_.track_index1;//边的标号
  int match_index0_mt = moving_object_.track_index1;
  int line_index0_cd = match_index0_cd;
  float delta_angle_line0[4];
  float min_delta_angle0 = 361;
  for (int m=0; m<4; m++)
  {
    //两个向量的夹角
    delta_angle_line0[m] = BaseFunction::Angle_line2(line4_cd[line_index0_cd], line4_mt[m]);
    if (delta_angle_line0[m] < min_delta_angle0)
    {
      min_delta_angle0 = delta_angle_line0[m];//找到夹角最小边
      match_index0_mt = m;
    }
  }
  int match_index1_cd = BaseFunction::value_in_threshod_int(match_index0_cd+1, 0,3);
  int match_index1_mt = BaseFunction::value_in_threshod_int(match_index0_mt+1, 0,3);
  int point_index_pre2cd = match_index0_cd - match_index0_mt;
  return point_index_pre2cd;
}

float ObjectMatching::CalcSimilarValue(CandidateObject candidate_object, MovingObject moving_object)
{
  float similar_value = 0;

  int index1_cd = candidate_object.shape.polar4[1].point_index;
  int index2_cd = candidate_object.shape.polar4[2].point_index;

  /// Calculate the distance between detect and tracking object in current vehicle coordinate
  float dis_x_center = candidate_object.center_point.x - moving_object.center_point_pre.x;
  float dis_y_center = candidate_object.center_point.y - moving_object.center_point_pre.y;
  float dis_center = sqrt(dis_x_center*dis_x_center + dis_y_center*dis_y_center);

  CvPoint2D32f temp_point_cd = candidate_object.center_point;
  if(candidate_object.shape.polar_state == 2)//TODO:看到一条边,匹配分析点定为中心点,其实在视角发生变化时也会突变
  {
    temp_point_cd.x = (candidate_object.shape.point4[index1_cd].x + candidate_object.shape.point4[index2_cd].x)/2;
    temp_point_cd.y = (candidate_object.shape.point4[index1_cd].y + candidate_object.shape.point4[index2_cd].y)/2;
  }

  bool is_region_radius = false;//是不是在匹配圆区域内
  bool is_region_track = false;//跟踪点是不是在匹配四边形区域内
  bool is_region_center = false;//中心点是不是在匹配四边形区域内
  if (fabs(dis_x_center)<8 && fabs(dis_y_center)<25 )//初筛选区域
  {
    is_region_track = Point_in_region(temp_point_cd, moving_object.match_range.range_rect4);
    is_region_center = Point_in_region(candidate_object.center_point, moving_object.match_range.range_rect4);
    if ( dis_center <2 && moving_object.object_type < 7)
      is_region_radius = true;
  }

  //可以进行匹配分析的目标,满足任一条件都可以
  if (is_region_center || is_region_track || is_region_radius)
  {
    float delta_x = dis_x_center;
    float delta_y = dis_y_center;

    //关键步骤:计算相似值
    float weight_x = 0.5;//similar_weight_.similar_weight_x;
    float weight_y = 0.5;//similar_weight_.similar_weight_y;
    similar_value = weight_x/(1 + fabs(delta_x)) + weight_y/(1 + fabs(delta_y));
  }

  return similar_value;
}

void ObjectMatching::InitParam()
{
  similar_value_matrix_ = Eigen::MatrixXf::Zero(moving_object_num,candidate_object_num);
  match_state_mt_ = std::vector<MatchState>(moving_object_num,init_matchstate_);
  match_state_cd_ = std::vector<MatchState>(candidate_object_num,init_matchstate_);
}


void ObjectMatching::CalcMatchMatrix(const vector<CandidateObject>* const candidate_object_vevctor_, const vector<MovingObject> * const moving_object_vector_)
{
  //************** calculate similar value **************//
  /*****************
   * 计算目标匹配相似值
   *****************/
  for (int j = 0; j < moving_object_num; j++){//遍历跟踪目标序列
    MovingObject temp_moving_object = moving_object_vector_->at(j);
    for (int i=0; i<candidate_object_num; i++){//遍历当前检测目标
      CandidateObject temp_candidate = candidate_object_vevctor_->at(i);
      float similar_value_mt = CalcSimilarValue(temp_candidate, temp_moving_object);
      similar_value_matrix_(j,i) = similar_value_mt;
      if(similar_value_mt > 0.1){
        this->match_state_mt_.at(j).similar_num++;
        this->match_state_mt_.at(j).similar_index_vector.push_back(i);
        this->match_state_cd_.at(i).similar_num++;
        this->match_state_cd_.at(i).similar_index_vector.push_back(j);
      }
    }
  }
  //************** calculate similar value **************//

  if(0)//打印匹配状态
  {
    std::cout<<"moving_object_match_status..."<<std::endl;
    for(int mt=0;mt<moving_object_num;++mt){
      std::cout<<mt<<" "<<this->match_state_mt_.at(mt).is_match<<" "<<this->match_state_mt_.at(mt).similar_num<<" ";
      for(int i=0;i<this->match_state_mt_.at(mt).similar_index_vector.size();++i){
        std::cout<<this->match_state_mt_.at(mt).similar_index_vector[i]<<" ";
      }
      std::cout<<std::endl;
    }

    std::cout<<"candidate_object_match_status..."<<std::endl;
    for(int mt=0;mt<candidate_object_num;++mt){
      std::cout<<mt<<" "<<this->match_state_cd_.at(mt).is_match<<" "<<this->match_state_cd_.at(mt).similar_num<<" ";
      for(int i=0;i<this->match_state_cd_.at(mt).similar_index_vector.size();++i){
        std::cout<<this->match_state_cd_.at(mt).similar_index_vector[i]<<" ";
      }
      std::cout<<std::endl;
    }
  }

  //打印相似度矩阵
//  std::cout<<"similarvalue_matrix_mt:"<<std::endl;

  /********************
   * 寻找相似度矩阵中最大值
   ********************/
  //1)按行寻找
  for(int idx = 0;idx<similar_value_matrix_.rows();++idx){
    Eigen::MatrixXf::Index maxIndex;
    float maxValue = similar_value_matrix_.row(idx).maxCoeff(&maxIndex);
    if(maxValue > 0){
      this->match_state_mt_.at(idx).match_index = maxIndex;
      this->match_state_cd_.at(maxIndex).max_similar_num++;
      this->match_state_cd_.at(maxIndex).max_similar_index_vector.push_back(idx);
    }
  }
  //2)按列寻找
  for(int idx = 0;idx<similar_value_matrix_.cols();++idx){
    Eigen::MatrixXf::Index maxIndex;
    float maxValue = similar_value_matrix_.col(idx).maxCoeff(&maxIndex);
    if(maxValue > 0){
      this->match_state_cd_.at(idx).match_index = maxIndex;
      this->match_state_mt_.at(maxIndex).max_similar_num++;
      this->match_state_mt_.at(maxIndex).max_similar_index_vector.push_back(idx);
    }
  }

//
//  std::cout<<"maximum moving_object_match_status..."<<std::endl;
//  for(int mt=0;mt<moving_object_num;++mt){
//    int idx = match_state_mt->at(mt).match_index;
//    std::cout<<mt<<" "<<idx<<" ";
//    if(idx>-1){//有匹配
//      std::cout<<match_state_cd->at(idx).max_similar_num<<std::endl;
//      for(auto k:match_state_cd->at(idx).max_similar_index_vector)
//      {
//        std::cout<<k<<" ";
//      }
//    }
//    std::cout<<std::endl;
//  }
}

void ObjectMatching::ProcessMatch(vector<CandidateObject>* candidate_object_vevctor_,vector<MovingObject>* moving_object_vector_,
                                  std::vector<MatchState>& match_state_cd,std::vector<MatchState>& match_state_mt, bool show_match)
{
  assert(0!=match_state_cd.size());
  assert(0!=match_state_mt.size());
  candidate_object_num = match_state_cd.size();
  moving_object_num = match_state_mt.size();

  if(show_match){
    this->ShowMatchAnalysisMap(*candidate_object_vevctor_,*moving_object_vector_);
  }

  InitParam();

  //匹配分析,输入：当前检测目标、跟踪上目标,更新match_state_cd_和match_state_mt_两个向量中的匹配向量
  CalcMatchMatrix( candidate_object_vevctor_, moving_object_vector_);

  for(const auto& obj:match_state_mt_){
    cout<<obj.match_index<<endl;
  }

  //接下来开始最终匹配分析,确认最终是否匹配以及选择最优匹配
  for(int j = 0;j<moving_object_vector_->size();++j){
    //1)找到其最佳匹配对应检测目标索引
    int match_idx = match_state_mt_.at(j).match_index;
    if(match_idx<0)
      continue;

    //2)判断该检测目标匹配情况
    if(1 == this->match_state_cd_.at(match_idx).max_similar_num){//刚好一对一匹配,完全匹配
      match_state_mt.at(j).is_match = true;
      match_state_mt.at(j).match_index = match_idx;
      match_state_cd.at(match_idx).is_match = true;
    }
    else if(this->match_state_cd_.at(match_idx).max_similar_num > 1){
      if((this->match_state_cd_.at(match_idx).match_index == j)&&(!this->match_state_cd_.at(match_idx).is_match)){//最大匹配是自己
        match_state_mt.at(j).is_match = true;
        match_state_mt.at(j).match_index = match_idx;
        match_state_cd.at(match_idx).is_match = true;
      }
      else{//TODO:not properly in this case
        match_state_mt.at(j).is_match = false;
        match_state_mt.at(j).match_index = -1;
      }
    }
  }//end for(int j = 0;j<moving_object_vector_->size();++j)
}


void ObjectMatching::ShowMatchAnalysisMap(const vector<CandidateObject>& candidate_object_vevctor, const vector<MovingObject>& moving_object_vector)
{
  //1)可视化目标点,可视化当前检测目标——绿色
  //和已经跟踪上的目标——黄色,分析匹配准则
  cv::Mat match_img = cv::Mat::zeros(map_height,map_width,CV_8UC3);
  dynamic_object_tracking::VisualizerUtils* vis_result = dynamic_object_tracking::VisualizerUtils::getInstance();
  for(const auto& candidate_obj:candidate_object_vevctor){
    vis_result->DrawEgoPoints(match_img,candidate_obj.center_point.x,candidate_obj.center_point.y,cv::Scalar(0,255,0));
  }

  for(const auto& moving_obj:moving_object_vector){
    vis_result->DrawEgoPoints(match_img,moving_obj.center_point_pre.x,moving_obj.center_point_pre.y,cv::Scalar(0,255,255));
  }

  Mat img_show = match_analysis_map_(Rect(0,0,map_width,map_height));
  match_img.copyTo(img_show);
  vis_result->DrawEgoVehicle(img_show);
  cv::namedWindow("match_analysis_image",CV_WINDOW_NORMAL);
  cv::imshow("match_analysis_image",match_analysis_map_);
}
