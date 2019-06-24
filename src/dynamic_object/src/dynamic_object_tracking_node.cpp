/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :
 * Created On   :2019-6-9
 * Descriptoin  : Lidar only for dynamic object detection and tracking using lab ros topics
 * References   :
======================================================================*/
//C++
#include <cmath>
#include <fstream>
#include <list>
#include <iomanip>
//OpenCV
#include <opencv/cv.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <glog/logging.h>
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "transform/rigid_transform.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "iv_dynamicobject_msgs/moving_target_send.h"
#include "iv_slam_ros_msgs/TraversableArea.h";
//load roadmap messages
#include "lanelet_map_msgs/LaneletMap.h"
//this project headers
#include "StructMovingTargetDefine.h"
#include "tracker/MovingTargetTrack.h"

static bool run_dynamic_map_tracker = false;//是否运行dynamic_map独立模块版本
static bool use_sensor_fusion_output = false;
static bool is_show_result = true;

static OGMProperty ogm_property(OGM_WIDTH, OGM_HEIGHT, OFFSET_Y, OGM_RESOLUTION);
static PolarProperty polar_property(60.0, 0.5, 360.0, 1.0);

void toTraversableAreaMsg(const cv::Mat& ogm_img,
                          const sensor_driver_msgs::OdometrywithGps& odometry_in,
                          bool isLateralMoving,
                          iv_slam_ros_msgs::TraversableArea& traversable_msg)
{
  if (!isLateralMoving) {
    traversable_msg.valid = false;
    return;
  }

  //1) Set basic properties
  traversable_msg.width = ogm_img.cols;
  traversable_msg.height = ogm_img.rows;
  traversable_msg.resolution = 0.2;
  traversable_msg.triD_submap_pose_image_index_x = 201;
  traversable_msg.triD_submap_pose_image_index_y = 251;

  //2) Set current odometry pose property
  traversable_msg.triD_submap_pose.position.x = odometry_in.gps.longitude;
  traversable_msg.triD_submap_pose.position.y = odometry_in.gps.latitude;
  traversable_msg.triD_submap_pose.position.z = odometry_in.gps.altitude;
  traversable_msg.triD_submap_pose.orientation = odometry_in.odometry.pose.pose.orientation;

  //3) Set OGM cells
  if (ogm_img.type() != CV_8UC1)
    throw std::logic_error("function toTraversableAreaMsg: wrong img_ogm_origin_ type, should be CV_8UC1");

  traversable_msg.cells.resize(traversable_msg.width*traversable_msg.height, 0);
  for (int i = 0; i < ogm_img.rows; ++i) {
    const uchar* data = ogm_img.ptr<uchar>(i);
    for (int j = 0; j < ogm_img.cols; ++j) {
      int value = data[j]; // Get ogm image pixel value
      if (value == 255) {
        traversable_msg.cells[(traversable_msg.height - i - 1)*traversable_msg.width + j] = 2;
      }
    }
  }
  traversable_msg.valid = true;//是横向动态目标,告诉规划切换至我这边发布的单帧栅格图
}

class PostProcess
{
public:
  typedef std::pair<double, sensor_driver_msgs::OdometrywithGps> TimeOdometryPair;

  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
  processthread_(NULL),
  processthreadfinished_ (false),
  vehicle_speed_(0.0),
  is_open_dynamic_obs_det_(true),
  moving_target(ogm_property, polar_property)
  {
    init();
  }
  ~PostProcess()
  {
    lidarOdoms_.stopQueue();
    gpsdatas_.stopQueue();
    dynamicMapMsgs_.stopQueue();
    processthreadfinished_ = true;
    processthread_->join();
  }

  void init()
  {
    //初始化配置参数
    this->initParameters();
    moving_target.Init(is_show_result);

    //1)确定使用独立模块,还是直接接收点云
    if(run_dynamic_map_tracker){
      ROS_INFO("Use dynamic map tracking input...");
      subDynamicMap_ = nodehandle_.subscribe<iv_dynamicobject_msgs::DynamicMap>
      ("/dynamic_map",1,boost::bind(&PostProcess::dynamicmapHandler,this,_1));
    }
    else{
      subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
      ("/lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云
    }
    //2)确定使用GPS融合定位,还是雷达里程计
    if(use_sensor_fusion_output){
      ROS_INFO("Use use_sensor_fusion_output pose...");
      subGps_ = nodehandle_.subscribe<sensor_driver_msgs::GpswithHeading>
      ("/sensor_fusion_output", 1, boost::bind(&PostProcess::gpsHandler,this,_1));//
    }
    else{
      subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
      ("/lidar_odometry_to_earth", 5, boost::bind(&PostProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉
    }

    subLaneLetMap_ = nodehandle_.subscribe<lanelet_map_msgs::Way>("/topology_global_path",1, boost::bind(&PostProcess::LaneLetMapHandler,this,_1));

    //3)发布消息
    pub_target_send = nodehandle_.advertise<iv_dynamicobject_msgs::moving_target_send>("/MovingTarget",1);
    pub_clustered_rgb_cloud_ = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/clustered_rgb_cloud", 1);
    pub_traversable_area_ = nodehandle_.advertise<iv_slam_ros_msgs::TraversableArea>("/MovingTargetMap", 2);

    // Begin main process
    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }

  void initParameters()//初始化配置参数
  {
    is_show_result = nodehandle_.param("is_display_all",true);
    cout<<"is_show_result "<<is_show_result<<endl;
  }

  void gpsHandler(const sensor_driver_msgs::GpswithHeading::ConstPtr& gps)  //雷达里程计
  {
    ROS_INFO("<dynamicobject> GPS data callback...");
    gpsdatas_.Push(gps);
    //std::cout<<gps->gps.longitude<<std::endl;
  }

  void LaneLetMapHandler(const lanelet_map_msgs::Way::ConstPtr& lanelet_msgs)
  {
    this->is_open_dynamic_obs_det_ = lanelet_msgs->open_dynamic_obs_det;
  }

  void laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)
  {
    //      ROS_INFO("<dynamicobject> laserOdometryHandler data callback...");
    double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
    geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

    Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
    Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
        laserOdometry->odometry.pose.pose.position.y,
        laserOdometry->odometry.pose.pose.position.z);

    transform::Rigid3d transformodometry(translation,roatation);
    lidarOdoms_.Push(common::make_unique<TimeOdometryPair>(timeOdometry, *laserOdometry));
  }

  void dynamicmapHandler(const iv_dynamicobject_msgs::DynamicMap::ConstPtr& dynamic_map_msg)
  {
    dynamicMapMsgs_.Push(dynamic_map_msg);
    if(dynamicMapMsgs_.Size()>2)
      dynamicMapMsgs_.Pop();
  }

  void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) //点云数据
  {
//    ROS_INFO("<dynamicobject> laserOdometryHandler data callback...");
    double timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();
    //LOG(INFO)<<std::fixed<<std::setprecision(3)<<"cloudtime:"<<timeLaserCloudFullRes;
    //LOG(INFO)<<"starttime"<<ros::Time::now().toSec() - timeLaserCloudFullRes;
    lidarCloudMsgs_.Push(laserCloudFullRes2);
    if(lidarCloudMsgs_.Size()>2)
      lidarCloudMsgs_.Pop();
  }

  void analysisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr inputcloud,
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& outputclouds,std::vector<pcl::PointXYZI>& lidarpropertys)
  {
    /////////总的点云中可能包含了几组独立的点云数据，对发过来的点云进行处理，将每一组点云都提取出来///////////
    int cloudnum = inputcloud->size() % 16;//包含的点云包数目
    vector<int> startnum;
    for(int i =0;i<cloudnum;i++)
    {
      pcl::PointXYZI originpoint;
      int flag = (*inputcloud)[inputcloud->size()-cloudnum+i].range;//每一包点云的第一个点的位置
      (*inputcloud)[inputcloud->size()-cloudnum+i].range = -0.5;
      originpoint.x = (*inputcloud)[inputcloud->size()-cloudnum+i].x;//每一包点云中对应的雷达在车体坐标系的x
      originpoint.y = (*inputcloud)[inputcloud->size()-cloudnum+i].y;////每一包点云中对应的雷达在车体坐标系的y
      originpoint.z = (*inputcloud)[inputcloud->size()-cloudnum+i].z;////每一包点云中对应的雷达在车体坐标系的z
      originpoint.intensity = (*inputcloud)[inputcloud->size()-cloudnum+i].azimuth;//每一包点云中对应的雷达线束
      startnum.push_back(flag);
      lidarpropertys.push_back(originpoint);
    }
    for(int i = 0;i < startnum.size();i++)
    {
      int length;
      pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloudptr(new pcl::PointCloud<pcl::PointXYZI>);//每一包点云

      if(i == startnum.size()-1)
      {
        length = inputcloud->size() - cloudnum - startnum.at(i);
      }
      else
      {
        length = startnum.at(i+1) - startnum.at(i);
      }

      lasercloudptr->insert(lasercloudptr->begin(),inputcloud->begin()+startnum.at(i),inputcloud->begin()+startnum.at(i)+length);
      outputclouds.push_back(lasercloudptr);
    }
  }


  void ToMovingTargetMsg(const MovingTargetSend& target_send, iv_dynamicobject_msgs::moving_target_send* moving_target_msg)
  {
    moving_target_msg->target_num = target_send.target_num;
    moving_target_msg->target.clear();
    for(auto iter = target_send.target_output.begin();iter!=target_send.target_output.end();++iter){
      iv_dynamicobject_msgs::moving_target target_send;
      //发送目标矩形框
      target_send.line_num = iter->line_num;
      target_send.line_point.clear();
      for (int i = 0; i < iter->line_num; ++i)
      {
        iv_dynamicobject_msgs::Points temp_point;
        temp_point.x = iter->line_point[i].x;
        temp_point.y = iter->line_point[i].y;
        temp_point.z = 0;
        target_send.line_point.push_back(temp_point);//相对于目标中心点坐标
      }

      target_send.center_point.x = iter->center_point.x;
      target_send.center_point.y = iter->center_point.y;
      target_send.center_point.z = 0;
      target_send.object_high = iter->object_high;
      target_send.object_type = iter->object_type;
      target_send.ID_number = iter->ID_number;
      target_send.is_updated = iter->is_updated;
      target_send.tracked_times = iter->tracked_times;
      target_send.dangerous_level = iter->dangerous_level;

      //发送预测信息
      target_send.predict_num = iter->predict_info.size();
      target_send.predict_traj.clear();
      for (const auto obj:iter->predict_info) {
        iv_dynamicobject_msgs::Predict_traj temp_traj;
        temp_traj.time_stamp = obj.time_stamp;
        temp_traj.point.x = obj.point.x;
        temp_traj.point.y = obj.point.y;
        temp_traj.v_x = obj.v_x;
        temp_traj.v_y = obj.v_y;
        target_send.predict_traj.push_back(temp_traj);
      }

      //发送历史信息
      target_send.history_num = iter->history_num;
      target_send.history_traj.clear();
      for (int i = 0; i < target_send.history_num; ++i) {
        iv_dynamicobject_msgs::History_traj temp_traj;
        temp_traj.time_stamp = iter->history_traj[i].history_time;
        temp_traj.center_point.x = iter->history_traj[i].history_center.x;
        temp_traj.center_point.y = iter->history_traj[i].history_center.y;
        temp_traj.center_point.z = 0;
        //发送矩形框四个点
        temp_traj.line_num = 4;
        temp_traj.line_point.clear();
        for (int j = 0; j < temp_traj.line_num; ++j) {
          iv_dynamicobject_msgs::Points temp_point;
          temp_point.x = iter->history_traj[i].history_rect.point4[j].x;
          temp_point.y = iter->history_traj[i].history_rect.point4[j].y;
          temp_point.z = 0;
          temp_traj.line_point.push_back(temp_point);
        }
        target_send.history_traj.push_back(temp_traj);
      }
      moving_target_msg->target.push_back(target_send);
    }//end for(auto iter...
  }

  void ToMovingTargetMsg(const MovingTargetOutput& target_output, iv_dynamicobject_msgs::moving_target_send* moving_target_msg)
  {
    moving_target_msg->target_num = target_output.target_num;
    moving_target_msg->target.clear();
    //遍历每个动态目标
    for(auto iter = target_output.target_output.begin();iter!=target_output.target_output.end();++iter){
      iv_dynamicobject_msgs::moving_target target_send;
      target_send.ID_number = iter->target_ID;
      target_send.center_point.x = iter->center_pt_ab.x;
      target_send.center_point.y = iter->center_pt_ab.y;
      //以下属性暂时不用
      target_send.center_point.z = 0;
      target_send.object_high = 0.0;
      target_send.object_type = 0;
      target_send.is_updated = 1;
      target_send.tracked_times = -1;
      target_send.dangerous_level = 0;

      //发送目标矩形框
      target_send.line_num = 4;//矩形框四个点
      target_send.line_point.clear();
      for (int i = 0; i < 4; ++i)
      {
        iv_dynamicobject_msgs::Points temp_point;
        temp_point.x = iter->line_point[i].x;
        temp_point.y = iter->line_point[i].y;
        temp_point.z = 0;
        target_send.line_point.push_back(temp_point);
      }

      //发送预测信息
      target_send.predict_num = iter->predict_traj.size();
      target_send.predict_traj.clear();
      for(const auto obj:iter->predict_traj){
        iv_dynamicobject_msgs::Predict_traj temp_traj;
        temp_traj.time_stamp = obj.time_stamp;
        temp_traj.point.x = obj.point.x;
        temp_traj.point.y = obj.point.y;
        temp_traj.v_x = obj.v_x;
        temp_traj.v_y = obj.v_y;
        target_send.predict_traj.push_back(temp_traj);
      }

      //发送历史信息
      target_send.history_num = iter->history_info.size();
      target_send.history_traj.clear();
      for (int i = 0; i < target_send.history_num; ++i)
      {
        iv_dynamicobject_msgs::History_traj temp_traj;
        temp_traj.time_stamp = iter->history_info[i].time_stamp;
        temp_traj.center_point.x = iter->history_info[i].history_center_ab.x;
        temp_traj.center_point.y = iter->history_info[i].history_center_ab.y;
        temp_traj.center_point.z = 0;
        //发送矩形框四个点
        temp_traj.line_num = 4;
        temp_traj.line_point.clear();
        for (int j = 0; j < temp_traj.line_num; ++j)
        {
          iv_dynamicobject_msgs::Points temp_point;
          temp_point.x = iter->history_info[i].history_rect_ab.point4[j].x;
          temp_point.y = iter->history_info[i].history_rect_ab.point4[j].y;
          temp_point.z = 0;
          temp_traj.line_point.push_back(temp_point);
        }
        target_send.history_traj.push_back(temp_traj);
      }
      moving_target_msg->target.push_back(target_send);
    }//end for(auto iter...
  }


  void process()
  {
    long long frame_counter = 0;
    State_Vehicle ego_veh_state_gps;
    State_Vehicle ego_veh_state_lidar;
    while (!processthreadfinished_&&ros::ok()) {
      timer t_total;
      double lidartime = 0.0;
      pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云
      iv_dynamicobject_msgs::DynamicMapConstPtr dynamic_map_msg = nullptr;

      /// -------According to use dynamic map or not to decide whether subscribe lidar cloud-----------
      if (!run_dynamic_map_tracker) {
        const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(common::FromSeconds(0.1));
        if(cloudmsg == nullptr)
        {
          ROS_WARN("Wait for data...");
          continue;
        }
        lidartime = cloudmsg->header.stamp.toSec();
        pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
      }
      else {//获取点云处理过的栅格图
        dynamic_map_msg = dynamicMapMsgs_.PopWithTimeout(common::FromSeconds(0.1));
        if(dynamic_map_msg == nullptr){
          continue;
        }
        //秒为单位的雷达时间戳
        lidartime = dynamic_map_msg->header.stamp.toSec();
      }


      memset(&ego_veh_state_gps, 0, sizeof(State_Vehicle));
      memset(&ego_veh_state_lidar, 0, sizeof(State_Vehicle));
      ego_veh_state_gps.is_update = false;
      ego_veh_state_lidar.is_update = false;

      //雷达里程计同步
      if (lidarOdoms_.Size()>0 && frame_counter >1) {
        std::cout<<"LidarOdom data synchronizing..."<<std::endl;
        auto timeposepair = lidarOdoms_.Pop();//需要雷达里程计信息时需要，否则可以注释掉
        if(timeposepair==nullptr)
          continue;
        while((timeposepair->first-lidartime)<-0.005)
          timeposepair = lidarOdoms_.Pop();
        if(timeposepair->first-lidartime>0.005){ //如果雷达里程计时间晚于雷达点云时间，则跳到下一点云
          lidarOdoms_.Push_Front(std::move(timeposepair));
          continue;
        }

        ego_veh_state_lidar.is_update = true;
        //std::cout<<"lidartime: "<<lidartime<<"odometry time: "<<timeposepair->first<<std::endl;
        current_odometry_ = timeposepair->second;
        geometry_msgs::Quaternion geoQuat = timeposepair->second.odometry.pose.pose.orientation;
        Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
        Eigen::Vector3d translation=Eigen::Vector3d(timeposepair->second.odometry.pose.pose.position.x,
                                                    timeposepair->second.odometry.pose.pose.position.y,
                                                    timeposepair->second.odometry.pose.pose.position.z);
        Eigen::Vector3d angle = transform::toRollPitchYaw(roatation);

        if(ego_veh_state_lidar.is_update)
        {
          ego_veh_state_lidar.time_stamp = timeposepair->first;
          ego_veh_state_lidar.fForwardVel = this->vehicle_speed_;
          ego_veh_state_lidar.global_position.dLng = translation[0];
          ego_veh_state_lidar.global_position.dLat = translation[1];
          ego_veh_state_lidar.global_position.heading = 360 - angle[2]*180/pi;//车体y轴(y轴向前)与正北方向的夹角,顺时针
          if(ego_veh_state_lidar.global_position.heading>=360)
            ego_veh_state_lidar.global_position.heading = ego_veh_state_lidar.global_position.heading - 360;
        }
      }

      //GPS信息同步
      if (gpsdatas_.Size()>0 && frame_counter>1) {
        std::cout<<"GPS data synchronizing..."<<std::endl;
        sensor_driver_msgs::GpswithHeading::ConstPtr gpsdata;
        gpsdata = gpsdatas_.Pop();
        std::cout<<"lidartime: "<<setprecision(13)<<lidartime<<" gps time: "<<gpsdata->gps.header.stamp.toSec()<<std::endl;
        while ((gpsdata->gps.header.stamp.toSec() - lidartime)<-0.050) {//GPS时间在点云之前,相当于等待GPS时间和雷达时间保持一致
          gpsdata = gpsdatas_.Pop();
        }
        if (gpsdata == nullptr) {
          ROS_INFO("No gpsdata!");
          continue;
        }
        if ((gpsdata->gps.header.stamp.toSec() - lidartime) > 0.050) { //如果GPS时间晚于雷达点云时间，则跳到下一点云
          gpsdatas_.Push_Front(gpsdata);
          continue;
        }
        ego_veh_state_gps.is_update = true;
        if(ego_veh_state_gps.is_update)
        {
          ego_veh_state_gps.time_stamp = gpsdata->gps.header.stamp.toSec();
          ego_veh_state_gps.fForwardVel = this->vehicle_speed_;
          ego_veh_state_gps.global_position.pitch = gpsdata->pitch;//俯仰角
          ego_veh_state_gps.global_position.roll = gpsdata->roll;//侧倾角
          ego_veh_state_gps.global_position.heading = 360 - gpsdata->heading;//TODO:什么方向？

          if(ego_veh_state_gps.global_position.heading>=360)
            ego_veh_state_gps.global_position.heading = ego_veh_state_gps.global_position.heading - 360;

          ego_veh_state_gps.global_position.dAld = 0;//gpsdata.;
          ego_veh_state_gps.global_position.dLat = gpsdata->gps.latitude;
          ego_veh_state_gps.global_position.dLng = gpsdata->gps.longitude;
          BaseFunction::Position_Trans_From_ECEF_To_UTM(gpsdata->gps.latitude, gpsdata->gps.longitude, 0,0,
              &ego_veh_state_gps.global_position.dLng, &ego_veh_state_gps.global_position.dLat);
        }
      }

      ++frame_counter;
      std::cout<<"========================= Begin Process "<<frame_counter<<" ============================"<<std::endl;
      State_Vehicle ego_veh_state_current = ego_veh_state_lidar;//最终使用的就是这个

      static double time_pre_ = lidartime;
      double delta_t = lidartime - time_pre_;
      time_pre_ = lidartime;
      std::cout<<"[TIMER] lidar get delta_t---------------------------- "<<delta_t<<endl;//TODO:查看雷达数据延时情况,最好显示出来
      std::cout<<"[TIMER] lidar get time "<<t_total.elapsed()<<endl;

      //开始处理
      if (tempcloud->size() > 0 && ego_veh_state_current.is_update) {
        moving_target.SetBasicInput(lidartime, ego_veh_state_current, this->is_open_dynamic_obs_det_);
        moving_target.SetCloudInput(tempcloud);//读入点云
        moving_target.ProcessFrame();
        std::cout<<"[TIMER] total handle exclude visualizing time "<<t_total.elapsed()<<endl;
        moving_target.DrawResult();

        if(1)//给规划发送ROS信息
        {
          //1) Get moving target information to send to Planning Module
          MovingTargetSend target_result_send = moving_target.get_Moving_Target_Send();

          //2) Define send message
          iv_dynamicobject_msgs::moving_target_send moving_target_msg;
          moving_target_msg.header.frame_id = "global_earth_frame";
          moving_target_msg.header.stamp = ros::Time(lidartime);//秒制单位雷达时间戳,代表对接收到的这一帧雷达的处理
          this->ToMovingTargetMsg(target_result_send, &moving_target_msg);
          pub_target_send.publish(moving_target_msg);

          //3) Publish single frame grid map
          bool isLateralMoving = false;
          cv::Mat moving_target_map = moving_target.getMovingTargetMap(isLateralMoving, false, true);
          if (!moving_target_map.empty()) {
            cv::namedWindow("ogm_res", CV_WINDOW_NORMAL);
            cv::imshow("ogm_res", moving_target_map);
          }
          iv_slam_ros_msgs::TraversableArea traversable_area_msg;
          traversable_area_msg.header.frame_id = "global_earth_frame";
          traversable_area_msg.header.stamp = ros::Time(lidartime);//秒制单位雷达时间戳,代表对接收到的这一帧雷达的处理
          toTraversableAreaMsg(moving_target_map, current_odometry_, isLateralMoving, traversable_area_msg);
          pub_traversable_area_.publish(traversable_area_msg);
        }
        //可视化
        moving_target.Show_Result_2D(t_total.elapsed());//TODO:加入单独显示同步耗时、算法耗时和总耗时

        //可视化聚类点云
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_ptr = moving_target.getRGBClusteredCloud();
        clustered_cloud_ptr->header.frame_id = "/vehicle_link";
        pub_clustered_rgb_cloud_.publish(*clustered_cloud_ptr);
      }
      else {//出错检查
        bool lidar_update = true;
        if(tempcloud->points.size()<1)
          lidar_update = false;
        bool ego_update = true;
        if(!ego_veh_state_current.is_update)
          ego_update = false;
//        std::cout<<"lidartime "<<setprecision(13)<< lidartime<<"  lidartime "<<setprecision(13)<< lidartime<<std::endl;
        std::cout<<"lidarupdate: "<< boolalpha<<lidar_update<<" || "<<"  egoupdate: "<< ego_update<<std::endl;
      }
      std::cout<< "-------------------- t_total: " << t_total.elapsed() << endl;
      std::cout<<"========================= End Process==============================="<<std::endl<<std::endl<<std::endl;
    }//end while(!processthreadfinished_)
  }

protected:
  ros::Subscriber subLaserCloudFullRes_ ;//经过筛选且转换之后的点云
  ros::Subscriber subLaserOdometry_ ;
  ros::Subscriber subGps_ ;
  ros::Subscriber subECU_;//订阅车辆状态信息
  ros::Subscriber subDynamicMap_;
  ros::Subscriber subLaneLetMap_;

  ros::Publisher pub_target_send;
  ros::Publisher pub_clustered_rgb_cloud_;
  ros::Publisher pub_traversable_area_;// Publish TraversableArea msg for planning

  common::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;
  common::BlockingQueue<std::unique_ptr<TimeOdometryPair>> lidarOdoms_;
  common::BlockingQueue<sensor_driver_msgs::GpswithHeading::ConstPtr> gpsdatas_;
  common::BlockingQueue<iv_dynamicobject_msgs::DynamicMap::ConstPtr> dynamicMapMsgs_;

  sensor_driver_msgs::OdometrywithGps current_odometry_;
  float vehicle_speed_;//本车速度
  bool is_open_dynamic_obs_det_;
  ros::NodeHandle& nodehandle_;
  boost::thread* processthread_;
  bool processthreadfinished_;

  MovingTargetTrack moving_target;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamicobject");
  ros::NodeHandle nh;

  //get parameters
  ros::param::get("~run_dynamic_map_tracker", run_dynamic_map_tracker);
  ros::param::get("~use_sensor_fusion_output", use_sensor_fusion_output);

  PostProcess postprocess(nh);
  ros::spin();

  return 0;
}

