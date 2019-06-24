#include "MovingTargetTrack.h"
#include <ros/ros.h>
#include "utils/cloud_process_utils.h"
#include "iv_dynamicobject_msgs/PolarCell.h"
#include "iv_dynamicobject_msgs/OGMCell.h"

static bool is_open_dynamic_object = true;//根据路网属性决定是否需要发送动态障碍物检测结果
static bool _is_draw_ego_traj = true;// 是否绘制本车轨迹
using namespace std;
using namespace cv;

MovingTargetTrack::MovingTargetTrack(const OGMProperty& ogm_property, const PolarProperty& polar_property):
    time_stamp(0),
    elevated_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
    ground_cloud_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
    is_show_submodule_result_(true),
    ogm_property_(ogm_property),
    polar_property_(polar_property),
    target_detector(ogm_property_,polar_property_),
    fusion_object_vector(),
    moving_object_vector(),
    moving_object_vector_send(),
    frame_counter(0),
    total_cost_time(0.0),
    last_lateral_moving_target_(false),
    lateral_counter_(0)
{
  memset(&ego_veh_state_current, 0, sizeof(State_Vehicle));
  memset(&moving_target_send,0,sizeof(MovingTargetSend));

  //  ROS_WARN_STREAM("ogm map property is "<<ogm_property_.ogm_height_<<" "<<ogm_property_.ogm_width_<<" "<<ogm_property_.resolution_);
  //  ROS_WARN_STREAM("ogm map property is "<<ogm_property_.mapSize().width<<" "<<ogm_property_.mapSize().height<<" "<<ogm_property_.mapOffset());
  //  ROS_WARN_STREAM("ogm map property is "<<ogm_property_.mapCellNum());

  /**************初始化栅格地图相关变量*****************/
  refineogm_width = ogm_property_.ogm_width_;//80m
  refineogm_height = ogm_property_.ogm_height_;//100m
  refineogm_offset_y = ogm_property_.ogm_y_offset_;//50m
  refineogm_resolution = ogm_property_.resolution_;//0.2
  refineogmwidth_cell = ogm_property_.mapSize().width;//401
  refineogmheight_cell = ogm_property_.mapSize().height;//501
  refineogmcell_size = ogm_property_.mapCellNum();//321201

  rigid_refined_ogm_vec_.resize(refineogmcell_size);

  //绘制射线的一些参数
  virtual_line_resolution = polar_property_.circular_resolution_;//现在是1度一根射线
  virtual_line_num  = polar_property_.circularCellsNum();//360度除以1度得到360根

  // Resize the size of virtual line vector
  virtual_line_vec_.resize(virtual_line_num);

  //初始化栅格地图图像
  img_ogm = cvCreateImage(cvSize(refineogmwidth_cell, refineogmheight_cell),8,1);//401x501
  img_target_fusion_ = cv::Mat::zeros(refineogmheight_cell+100, refineogmwidth_cell, CV_8UC3);
  cvZero(img_ogm);
  /**************初始化栅格地图相关变量*****************/

  map_offset_y = ogm_property_.mapOffset();
  map_resolution = refineogm_resolution;
  ego_veh_position = ogm_property_.mapAnchor();

  //给ShowResult类初始化
  show_result.Init(refineogm_resolution, refineogm_offset_y);
  //可视化用
  vis_result_ = dynamic_object_tracking::VisualizerUtils::getInstance();
  vis_result_->Init(this->ogm_property_);
}

MovingTargetTrack::~MovingTargetTrack()
{
  cvReleaseImage(&img_ogm);
}

bool MovingTargetTrack::Init(bool is_show_submodule_result)
{
  multi_target_match.Init(refineogmwidth_cell, refineogmheight_cell, map_offset_y, refineogm_resolution,
                          virtual_line_num, virtual_line_resolution,false);
  target_motion_updating_.Init(refineogm_width, refineogm_height, refineogm_offset_y, refineogm_resolution,
                             virtual_line_num, virtual_line_resolution,is_show_submodule_result);
  return true;
}

void MovingTargetTrack::SetBasicInput(double time_stamp,const State_Vehicle& ego_veh_state_current, bool _is_open_dynamic_object)
{
  ++this->frame_counter;
  this->time_stamp = time_stamp;
  this->ego_veh_state_current = ego_veh_state_current;
  this->total_cost_time = 0.0;

  //历史车辆位姿
  this->ego_veh_state_history_.push_back(ego_veh_state_current);
  if(this->ego_veh_state_history_.size() > 50) {//最多50帧
    this->ego_veh_state_history_.pop_front();
  }
  is_open_dynamic_object = _is_open_dynamic_object;//根据路网决定是否打开动态障碍物
}

void MovingTargetTrack::SetCloudInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in)
{
  //传递点云共享指针
  this->cloud_input = cloud_in;
  float x_limit = ogm_property_.ogm_width_/2.0;
  float y_forward_limit = ogm_property_.ogmYForwardLim();
  float y_backward_limit = ogm_property_.ogmYBackwardLim();
  this->cloud_limit_ = sensors_fusion::CloudFilter(cloud_in, -x_limit, x_limit, y_backward_limit, y_forward_limit, -5.0, 5.0);
}

void MovingTargetTrack::ProcessFrame()
{
  //开始处理
  std::cout<<"***************frame_counter: "<<frame_counter<<"************************"<<std::endl;

  std::cout<<"-----------------Begin PreProcessNew--------------------------------"<<std::endl;
  timer t_preprocess;
  sensors_fusion::ogm_mapping::CreateOGM(*this->cloud_limit_, this->rigid_refined_ogm_vec_, this->ogm_property_);

  sensors_fusion::CloudPair<pcl::PointXYZI> obstacle_ground_cloud_pair_ =
      sensors_fusion::RemoveGround(this->cloud_limit_, this->ogm_property_, rigid_refined_ogm_vec_, 0.5);
  elevated_cloud_ptr_ = obstacle_ground_cloud_pair_.first;
  /// 创建直角坐标障碍物栅格地图
  CreateObstacleOGM(rigid_refined_ogm_vec_);

  /// 创建极坐标栅格地图
  polar_cells_vector_ = vector<Polar_Cell>(polar_property_.polarCellsNum());
  sensors_fusion::ogm_mapping::CreatePolarOGM(obstacle_ground_cloud_pair_.first, polar_cells_vector_, polar_property_);

  total_cost_time += t_preprocess.elapsed();
  std::cout<<"t_preprocess: "<<t_preprocess.elapsed()<<endl;
  std::cout<<"-----------------End PreProcessNew----------------------------------"<<std::endl;

  std::cout<<"-----------------Begin TargetDetection-----------------------------"<<std::endl;
  this->TargetDetection();//目标检测
  std::cout<<"-----------------End TargetDetection-------------------------------"<<std::endl;

  std::cout<<"-----------------Begin MultiTargetMatch----------------------------"<<std::endl;
  this->MultiTargetMatch();//目标匹配，跟踪
  std::cout<<"-----------------End MultiTargetMatch-------------------------------"<<std::endl<<std::endl;
  this->DetermineTargetSend();//目标发送
//  this->DetermineTargetSend(this->moving_object_vector);//目标发送
}

void MovingTargetTrack::ProcessMapFrame()
{
  //开始处理
  std::cout<<"***************frame_counter: "<<frame_counter<<"************************"<<std::endl;
  std::cout<<"-----------------Begin TargetDetection-----------------------------"<<std::endl;
  this->TargetDetection();//目标检测
  std::cout<<"-----------------End TargetDetection-------------------------------"<<std::endl;
  std::cout<<"-----------------Begin MultiTargetMatch----------------------------"<<std::endl;
  this->MultiTargetMatch();//目标匹配，跟踪
  std::cout<<"-----------------End MultiTargetMatch-------------------------------"<<std::endl<<std::endl;
  this->DetermineTargetSend();//目标发送
}

void MovingTargetTrack::CreateObstacleOGM(const std::vector<OGM_Cell>& ogm_cell_vec)
{
  cvZero(img_ogm);
  for (int i = 0; i < img_ogm->height;i++) {
    for (int j = 0; j < img_ogm->width;j++) {
      //将RIGIDNOPASSABLE属性的占据栅格在img_ogm上用白色点表示
      if (ogm_cell_vec[i*refineogmwidth_cell + j].type == CellProperty::RigidNoPassable) {
        img_ogm->imageData[(img_ogm->height - i - 1)*img_ogm->widthStep + j] = 255;//intensity;
      }
    }
  }
}

void MovingTargetTrack::TargetDetection()
{
  timer t_detect;
  BaseFunction::InitVirtualLine(virtual_line_vec_);
  /*************************************************************************************
   *                        对二值栅格地图完成目标检测功能
   * 检测结果存在fusion_object_vector中,绘制的射线相关信息存在virtual_line中
   ************************************************************************************/
  cv::Mat mat_ogm_img = cvarrToMat(this->img_ogm, false);
  target_detector.SetInput(frame_counter, mat_ogm_img, this->rigid_refined_ogm_vec_,
                           this->polar_cells_vector_, this->elevated_cloud_ptr_);
  target_detector.Detect(virtual_line_vec_);
  fusion_object_vector = target_detector.get_candidate_object_vector(true);
  target_detector.DrawResult(true);
  total_cost_time +=  t_detect.elapsed();
  cout<< " t_detect: " << t_detect.elapsed() << endl;
}

void MovingTargetTrack::MultiTargetMatch()
{
  timer t_track;
  /*************************************
   * 当前帧检测结果与已跟踪目标进行目标匹配
   ************************************/
  multi_target_match.MapInput(frame_counter,time_stamp, ego_veh_state_current, virtual_line_vec_);
  //利用当前帧检测结果和历史跟踪匹配的目标,进行新状态更新
  //即moving_object_vector包含了新的与当前帧哪些目标匹配上的信息
  multi_target_match.MatchProcess(img_ogm, &fusion_object_vector, &moving_object_vector);

  /*************************************
   * 利用匹配结果,进行Kalman跟踪状态更新
   ************************************/
  target_motion_updating_.MapInput(frame_counter,time_stamp, ego_veh_state_current);
  //目标状态更新
  target_motion_updating_.UpdateMotionState( img_ogm, &fusion_object_vector, &moving_object_vector);
  total_cost_time += t_track.elapsed();
  cout<< " t_track: " << t_track.elapsed() << endl;
}

void MovingTargetTrack::DetermineTargetSend()
{
  /*************************************
   * 动态目标发送给规划
   ************************************/
  //根据路网首先判断是否需要发送动态障碍物结果
  if(!is_open_dynamic_object){//需要清除发送,否则会遗留上一次的检测结果
    moving_object_vector_send.clear();
    moving_target_send.target_num = 0;
    moving_target_send.target_output.clear();
    moving_target_output_.target_num = 0;
    moving_target_output_.target_output.clear();
    return;
  }

  moving_object_vector_send.clear();
  for (size_t i=0; i < moving_object_vector.size(); ++i) {
    MovingObject temp_object = moving_object_vector.at(i);
    //只考虑运动目标,并且Kalman后验速度大于1
    if (temp_object.motion_behavior > 0 &&
        temp_object.object_type > 0 &&
        temp_object.motion_state_ab.v_post > 0.9&&
        temp_object.track_state.tracked_state > 0&&
        temp_object.center_pt.y > 0&&
        !(!temp_object.has_match&&(temp_object.track_state.tracked_state == 1))){
      if(temp_object.send_times == 0){//准备第一次发送
        if((temp_object.center_pt.y > 0 && temp_object.center_pt.y <= 3)&&
            (temp_object.center_pt.x >= -5 && temp_object.center_pt.x <= 5))
          continue;
      }
      if (temp_object.send_times > 5) {//已经发送超过5次了,后面一直保持发送
        temp_object.send_times++;
        moving_object_vector_send.push_back(temp_object);
      }
      else {//发送次数较少的,只将满足一定距离约束条件的目标发送
        if ((fabs(temp_object.center_pt.x)<=10)&& temp_object.center_pt.y>-2.0) {
          temp_object.send_times++;
          moving_object_vector_send.push_back(temp_object);
        }
      }
    }
    moving_object_vector.at(i) = temp_object;
  }

  moving_target_send.time_stamp = time_stamp;//接收到点云数据的时间戳
  moving_target_send.target_num = moving_object_vector_send.size();
  moving_target_send.target_output.clear();
  for (size_t i = 0; i < moving_object_vector_send.size(); ++i) {
    MovingObject temp_moving_target = moving_object_vector_send.at(i);
    Target_output2 temp_target_out;
    //将运动目标信息转变为最终发送目标信息
    BaseFunction::MovingTarget2Send(temp_moving_target, &temp_target_out);
    moving_target_send.target_output.push_back(temp_target_out);
  }

  /*
   * 新添加发送 20180710
   */
  moving_target_output_.target_num = moving_object_vector_send.size();
  moving_target_output_.target_output.clear();
  for(const auto obj:moving_object_vector_send) {
    TargetOutput temp_target_out;
    BaseFunction::MovingTarget2Send(obj, temp_target_out);
    moving_target_output_.target_output.push_back(temp_target_out);
  }
}

void MovingTargetTrack::DetermineTargetSend(vector<MovingObject>& moving_target_vector)
{
  //根据路网首先判断是否需要发送动态障碍物结果
  if(!is_open_dynamic_object){//需要清除发送,否则会遗留上一次的检测结果
    moving_object_vector_send.clear();
    moving_target_send.target_num = 0;
    moving_target_send.target_output.clear();
    moving_target_output_.target_num = 0;
    moving_target_output_.target_output.clear();
    return;
  }
  //针对比赛,只发送单个目标,并剔除跟踪目标序列中不可靠目标
  /*****************
   * 确定要发送的目标
   *****************/
  vector<MovingObject> moving_object_send_vector;//发送给规划的跟踪目标向量
  //1)初确定动态目标
  moving_object_send_vector.clear();
  for (int i=0; i<moving_target_vector.size(); ++i) {//遍历跟踪目标序列
    MovingObject temp_object = moving_target_vector.at(i);
    //选出运动目标
    if(temp_object.motion_behavior > 0 &&
        temp_object.motion_state_ab.v_post > 0.9 &&
        temp_object.track_state.tracked_state >= 1 &&
        temp_object.center_pt.y > 0&&
        !(!temp_object.has_match&&(temp_object.track_state.tracked_state == 2||temp_object.track_state.tracked_state == 1))){
      if(0 == temp_object.send_times) {
        if(fabs(temp_object.center_pt.x)<=1.8 &&
            (temp_object.center_pt.y >= -1.5 && temp_object.center_pt.y <= 4)){//如果目标第一次发送并且特别靠近本车,认为应该剔除
          moving_target_vector.erase(moving_target_vector.begin() + i);
          --i;
          continue;
        }
        else if((temp_object.center_pt.y > 0 && temp_object.center_pt.y <= 3)&&
            (temp_object.center_pt.x >= -5 && temp_object.center_pt.x <= 5)){
          continue;//不当做发送目标,也不剔除
        }
      }//end if(0 == temp_object.send_times)

      if(temp_object.send_times >= 5){//已经发送超过5次了,后面一直保持发送
        ++temp_object.send_times;
        moving_object_send_vector.push_back(temp_object);
      }
      else {
        if((fabs(temp_object.center_pt.x) < 8)&& temp_object.center_pt.y > 2.0){
          ++temp_object.send_times;
          moving_object_send_vector.push_back(temp_object);
        }
        else{
          temp_object.send_times = 0;
        }
      }
      moving_target_vector.at(i) = temp_object;
    }
  }

  //2)挑选出最可能比赛目标
  this->ChooseSendTargets(moving_object_send_vector);

  moving_target_send.time_stamp = time_stamp;//接收到点云数据的时间戳
  moving_target_send.target_num = this->moving_object_vector_send.size();
  moving_target_send.target_output.clear();
  for (int i=0; i<this->moving_object_vector_send.size(); i++)
  {
    MovingObject temp_moving_target = this->moving_object_vector_send.at(i);
    Target_output2 temp_target_out;
    //将运动目标信息转变为最终发送目标信息
    BaseFunction::MovingTarget2Send(temp_moving_target, &temp_target_out);
    moving_target_send.target_output.push_back(temp_target_out);
  }

  /*
   * 新添加发送 20180710
   */
  moving_target_output_.target_num = this->moving_object_vector_send.size();
  moving_target_output_.target_output.clear();
  for(const auto& obj:this->moving_object_vector_send) {
    TargetOutput temp_target_out;
    BaseFunction::MovingTarget2Send(obj, temp_target_out);
    moving_target_output_.target_output.push_back(temp_target_out);
  }
}

void MovingTargetTrack::ChooseSendTargets(const vector<MovingObject>& moving_object_send_vector)
{
  this->moving_object_vector_send.clear();
  if(moving_object_send_vector.size() > 0) {
    //最大发送次数目标
    auto max_send_times_iter = std::max_element(moving_object_send_vector.begin(), moving_object_send_vector.end(),
        [](const MovingObject& obj1, const MovingObject& obj2){
      return obj1.send_times < obj2.send_times;
    });

    if(max_send_times_iter->send_times >= 10) {
      this->moving_object_vector_send.push_back(*max_send_times_iter);//只发送该目标
      return;
    }

    //最远距离目标
    auto max_distance_iter = std::max_element(moving_object_send_vector.begin(), moving_object_send_vector.end(),
        [](const MovingObject& obj1, const MovingObject& obj2){
      return obj1.dis_veh_xy < obj2.dis_veh_xy;
    });
    if(max_distance_iter->dis_veh_xy >= 20) {
      this->moving_object_vector_send.push_back(*max_send_times_iter);//只发送该目标
      return;
    }

    //TODO:不行就发送所有目标,可能需要再加条件
    this->moving_object_vector_send = moving_object_send_vector;
  }
}


cv::Mat MovingTargetTrack::getMovingTargetMap(bool& isLateralMoving, bool isRemoveTarget, bool isFillMap)
{
  cv::Mat ogm_res = cvarrToMat(this->img_ogm, true);
  if (moving_object_vector_send.size() == 0) {
    isLateralMoving = false;
    ROS_WARN("No Moving Objects!==============");
    if (last_lateral_moving_target_ && lateral_counter_ < 50) {
      ++ lateral_counter_;
      isLateralMoving = true;
      return ogm_res;
    }
    else {
      last_lateral_moving_target_ = false;
      lateral_counter_ = 0;
      isLateralMoving = false;
      return cv::Mat();
    }
  }

  /// -----------------------Judge whether is lateral moving objects------------------
  //1) Convert global predict points to local points
  MovingObject object_temp = moving_object_vector_send[0];// Assume just one object
  Point2D temp_point_0, temp_point_20;
  //首尾预测轨迹点转车体坐标
  BaseFunction::point_global_to_local(object_temp.predict_info[0].point.x,
                                      object_temp.predict_info[0].point.y,
                                      ego_veh_state_current,
                                      &temp_point_0.x, &temp_point_0.y);
  BaseFunction::point_global_to_local(object_temp.predict_info[19].point.x,
                                      object_temp.predict_info[19].point.y,
                                      ego_veh_state_current,
                                      &temp_point_20.x, &temp_point_20.y);

  //2) 动态障碍物航向与局部车体坐标系航向夹角
  double delta_x = temp_point_20.x - temp_point_0.x;
  double delta_y = temp_point_20.y - temp_point_0.y;
  double dyn_local_heading = std::atan2(delta_x, delta_y);
  ROS_ERROR("========, %f, %f, %f, %f", delta_x, delta_y, dyn_local_heading*180/pi, fabs(dyn_local_heading));
  if (15.0 < fabs(dyn_local_heading*180/pi)) {//认为是横向,条件比较宽松
    isLateralMoving = true;
  }
  else {
    isLateralMoving = false;
    return cv::Mat();
  }

  last_lateral_moving_target_ = isLateralMoving;
  lateral_counter_ = 0;

  /// -----------------------Lateral moving targets using single frame for planning------------------
  //1) Get current OGM
  if (isRemoveTarget ^ isFillMap) {//Choose one method for lateral moving targets
    float length = sqrt(delta_x * delta_x + delta_y + delta_y);//预测距离

    for (int i = 0; i < moving_object_vector_send.size(); ++i) {
      MovingObject temp_object = moving_object_vector_send.at(i);
      //1) 矩形框四个点转像素坐标
      Point2D local_target_points[4];
      CvPoint contour_point4[4];//栅格地图点坐标
      for (int m = 0; m < 4; ++m) {
        Point_d temp_point_g = temp_object.contour_rect_ab.point4[m];
        //转车体坐标
        BaseFunction::point_global_to_local(temp_point_g.x, temp_point_g.y, ego_veh_state_current,
            &local_target_points[m].x, &local_target_points[m].y);
        //转栅格地图像素坐标
        Point2D temp_point;
        temp_point.x = local_target_points[m].x;
        temp_point.y = local_target_points[m].y;
        CvPoint2D32f temp_p;
        temp_p.x = (temp_point.x/map_resolution + ogm_res.cols/2);
        temp_p.y = (ogm_res.rows-1 - temp_point.y/map_resolution - map_offset_y);
        contour_point4[m] = cvPointFrom32f(temp_p);
      }
      //2) 计算中心点到矩形框顶点距离
      float dist1 = sqrt((local_target_points[0].x-local_target_points[1].x)*(local_target_points[0].x-local_target_points[1].x)+
          (local_target_points[0].y-local_target_points[1].y)*(local_target_points[0].y-local_target_points[1].y));
      float dist2 = sqrt((local_target_points[2].x-local_target_points[1].x)*(local_target_points[2].x-local_target_points[1].x)+
          (local_target_points[2].y-local_target_points[1].y)*(local_target_points[2].y-local_target_points[1].y));
      float radius = sqrt((dist1/2)*(dist1/2) + (dist2/2)*(dist2/2));
      cv::Point center_point((contour_point4[0].x + contour_point4[2].x)/2,
          (contour_point4[0].y + contour_point4[2].y)/2);

      //3) 使用圆形填充
      if (isRemoveTarget) {//Using remove target method
        cv::circle(ogm_res, center_point, 1.5*radius/map_resolution, Scalar(255), 0);
      }
      else {//Using fill predict location method
        int predict_num = length/(radius - 0.2);
        for (int i = 0; i < predict_num; ++i) {
          cv::Point real_center_pt;
          real_center_pt.x = center_point.x + ((delta_x*(radius - 0.2))/length)/map_resolution*i;
          real_center_pt.y = center_point.y - ((delta_y*(radius - 0.2))/length)/map_resolution*i;
          cv::circle(ogm_res, real_center_pt, radius/map_resolution, Scalar(255), -1);
        }
      }
    }
    return ogm_res;
  }
  else {
    throw std::logic_error("MovingTargetTrack::getMovingTargetMap function isRemove and fillMap must choose one method");
  }
}


void MovingTargetTrack::Show_Result_2D(double elapsed_time)
{
  timer t_show_2D;
  // Show dynamic objects information in image
  vis_result_->ShowText(img_target_fusion_, this->frame_counter, moving_target_send.target_num,
                        elapsed_time, ego_veh_state_current.fForwardVel);
  cv::namedWindow("img_target_fusion",CV_WINDOW_NORMAL);
  cv::imshow("img_target_fusion", img_target_fusion_);
  int key = cvWaitKey(5);

  // Whether save image
  bool is_save_fusion_img = false;
  if (is_save_fusion_img) {
    char filename[50];
    sprintf(filename, "%d_target_fusion_img.png", this->frame_counter);
    cv::imwrite(filename, img_target_fusion_);
  }
  cout<< " t_show_2DDDDDDD: "<<  t_show_2D.elapsed() << endl;
  cout<<" --------------t_total_time: "<<total_cost_time<<endl;
}

void MovingTargetTrack::DrawResult()
{
  timer t_draw_2D;

  // Image for drawing moving objects information
  cv::Mat img_moving_object;
  cv::Mat mat_ogm_img = cvarrToMat(this->img_ogm, false);
  cv::cvtColor(mat_ogm_img, img_moving_object, CV_GRAY2BGR);
  vis_result_->DrawEgoVehicle(img_moving_object);//绘制本车

  //TODO: 下面两句不能替换,原因未知
  show_result.Show_moving_vector(img_moving_object, moving_object_vector_send, ego_veh_state_current);
//  cout<<"fix bug ======================="<<endl;
//  for(int i = 0;i < moving_object_vector_send.size(); ++i){
//    MovingObject temp_object = moving_object_vector.at(i);
//    for(int m = 0; m < 4; ++m){
//      CvPoint2D32f temp_p;
//      Point_d temp_point_g = temp_object.contour_rect_ab.point4[i];
//      Point2D temp_point;
//      //转车体坐标
//      BaseFunction::point_global_to_local(temp_point_g.x, temp_point_g.y, ego_veh_state_current, &temp_point.x, &temp_point.y);
//      temp_p.x = (temp_point.x/map_resolution + 401/2);
//      temp_p.y = (501-1 - temp_point.y/map_resolution - map_offset_y);
//      CvPoint contour_point4 = cvPointFrom32f(temp_p);
//      cout<<"point======= "<<contour_point4.x<<" "<<contour_point4.y<<endl;
//    }
//  }
//  vis_result_->ShowMovingObjectVector(img_moving_object,&this->moving_object_vector_send);

  // 绘制在img_target_fusion图上
  // Clear last frame drawings
  img_target_fusion_.setTo(0);
  cv::Mat imgROI = img_target_fusion_(cv::Rect(0,100,img_moving_object.cols,img_moving_object.rows));
  img_moving_object.copyTo(imgROI);

  /************************************************
   * 画本车运动轨迹(相当于把历史帧本车全局位置投到当前帧下)
   ************************************************/
  if (_is_draw_ego_traj) {
    CvPoint ego_point0 = cvPoint(img_target_fusion_.cols/2, img_target_fusion_.rows - map_offset_y);
    for(const auto& veh_pos:ego_veh_state_history_){
      CvPoint2D32f ego_point1_32;
      float temp_x = 0;
      float temp_y = 0;
      BaseFunction::point_global_to_local(veh_pos.global_position.dLng, veh_pos.global_position.dLat,
                                          ego_veh_state_current, &temp_x, &temp_y);

      ego_point1_32.x = temp_x/map_resolution + img_target_fusion_.cols/2;
      ego_point1_32.y = img_target_fusion_.rows - temp_y/map_resolution - map_offset_y;

      CvPoint ego_point1 = cvPointFrom32f(ego_point1_32);
//      cvLine(img_target_fusion, ego_point0, ego_point1, cvScalar(0,255,0), 1,8,0);
      cv::circle(img_target_fusion_, ego_point1, 1, cvScalar(255,255,0),1,8,0);

      ego_point0 = ego_point1;
    }
  }


  //显示接收唐波路网属性标志,是否打开动态障碍物检测程序
  char text[50];
  sprintf(text, "Open? %d", (int)is_open_dynamic_object);
  cv::putText(img_target_fusion_, text, cv::Point(200,25),cv::FONT_HERSHEY_PLAIN, 1.3, cv::Scalar(0, 255, 0), 2);

  cout<< " t_draw_2D: "<<  t_draw_2D.elapsed() << endl;
  total_cost_time += t_draw_2D.elapsed();
  cout<<" --------------t_total_time: "<<total_cost_time<<endl;
}
