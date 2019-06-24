/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :
 * Created On   :
 * Descriptoin  : 利用KITTI数据,验证动态目标跟踪算法
 *               Needed topics: 1.sensor_msgs::Image
 *               2. sensor_msgs::PointCloud2
 *               3. GPS/IMU
 * References   :
======================================================================*/
//C++
#include <cmath>
#include <fstream>
#include <list>
#include <iomanip>
//OpenCV
#include <opencv2/opencv.hpp>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h> //image handler
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "transform/rigid_transform.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
#include "StructMovingTargetDefine.h"
#include "tracker/MovingTargetTrack.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/GpswithHeading.h"
#include "iv_dynamicobject_msgs/moving_target.h"
#include "iv_dynamicobject_msgs/moving_target_send.h"
#include "iv_dynamicobject_msgs/Predict_traj.h"
#include "iv_dynamicobject_msgs/History_traj.h"
#include "iv_dynamicobject_msgs/Points.h"
#include "iv_dynamicobject_msgs/Rectangle.h"
#include "iv_dynamicobject_msgs/TargetCar.h"
#include "iv_dynamicobject_msgs/RadarPoint.h"
#include "iv_dynamicobject_msgs/RadarData.h"

#include "utils/lidar_camera_projection_utils.h"
#include "utils/cloud_process_utils.h"

static bool is_show_result = true;
using namespace cv;

static Eigen::Matrix4f RT_velo_to_cam;
static Eigen::Matrix4f transform_needed;
static Eigen::Matrix4f R_rect_00;
static Eigen::MatrixXf camera_project_matrix;
static OGMProperty ogm_property(OGM_WIDTH, OGM_HEIGHT, OFFSET_Y, OGM_RESOLUTION);
static PolarProperty polar_property(60.0, 0.5, 360.0, 1.0);

class PostProcess
{
public:
  typedef std::pair<double,transform::Rigid3d> TimePosePair;

  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
  processthread_(NULL),
  processthreadfinished_ (false),
  project_matrix_(3,4),
  moving_target(ogm_property, polar_property)
  {
    init();
  }
  ~PostProcess()
  {
    imudatas_kitti_.stopQueue();
    hdldatas_kitti_.stopQueue();
    gpsdatas_kitti_.stopQueue();
    imagedatas_kitti_.stopQueue();
    processthreadfinished_ = true;
    processthread_->join();
    cv::destroyAllWindows();
  }

  void init()
  {
    RT_velo_to_cam<<7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03,
                    1.480249e-02,7.280733e-04,-9.998902e-01, -7.631618e-02,
                    9.998621e-01,7.523790e-03,1.480755e-02,  -2.717806e-01,
                    0.0, 0.0, 0.0, 1.0;
    transform_needed<<0.0, 1.0, 0.0, 0.0,
                      -1.0,0.0, 0.0, 0.0,
                      0.0, 0.0, 1.0, -1.73,
                      0.0, 0.0, 0.0, 1.0;
    R_rect_00<<9.999239e-01,9.837760e-03,-7.445048e-03, 0.0,
              -9.869795e-03,9.999421e-01,-4.278459e-03, 0.0,
               7.402527e-03,4.351614e-03,9.999631e-01, 0.0,
               0.0, 0.0, 0.0, 1.0;
    camera_project_matrix = Eigen::MatrixXf(3,4);
    camera_project_matrix<<7.215377000000e+02, 0.000000000000e+00, 6.095593000000e+02, 0.000000000000e+00,
                           0.000000000000e+00, 7.215377000000e+02, 1.728540000000e+02, 0.000000000000e+00,
                           0.000000000000e+00, 0.000000000000e+00, 1.0, 0.0;
    // Cloud to image transform matrix
    project_matrix_ = camera_project_matrix*RT_velo_to_cam*transform_needed;

    //init configure parameters
    moving_target.Init(is_show_result);
    image_transport::ImageTransport it(nodehandle_);
    subCamera_ = it.subscribeCamera("/kitti/camera_color_left/image_raw", 10, &PostProcess::CameraCallback, this);

//    subImage_ = nodehandle_.subscribe<sensor_msgs::Image>("/kitti_player/color/left/image_rect", 1, &PostProcess::ImageKittiHandler, this);
    subGpsKitti_ = nodehandle_.subscribe<sensor_msgs::NavSatFix>("/kitti/oxts/gps", 10, boost::bind(&PostProcess::gpsKittiHandler,this,_1));//
    subImu_ = nodehandle_.subscribe<sensor_msgs::Imu>("/kitti/oxts/imu",10,boost::bind(&PostProcess::ImuKittiHandler,this,_1));
    subVelodyne_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud",1,boost::bind(&PostProcess::VelodyneKittiHandler,this,_1));

    pub_target_send = nodehandle_.advertise<iv_dynamicobject_msgs::moving_target_send>("MovingTarget",1);
    pub_elevated_cloud_ = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZI>>("elevated_cloud",1);
    pub_clustered_rgb_cloud_ = nodehandle_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/clustered_rgb_cloud", 1);

    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }

  void CameraCallback(const sensor_msgs::ImageConstPtr& image_msg,
                      const sensor_msgs::CameraInfoConstPtr& camera_info_msg)
  {
    imagedatas_kitti_.Push(image_msg);
//    static bool initial_callback = true;
//    if(initial_callback)
//    {
//      const double* P = camera_info_msg->P.data();
//      for(int row = 0; row < 3;++row)
//        for(int col = 0; col < 4;++col){
//          int idx = row*4 + col;
//          project_matrix_(row,col)=P[idx];
//        }
//      ROS_WARN_STREAM("The raw camera projection matrix is:\n"<<project_matrix_<<"\n");
//      //change to correspond to our need projection matrix
//      project_matrix_ = project_matrix_*RT_velo_to_cam*transform_needed;
//      initial_callback = false;
//    }
  }

  void ImageKittiHandler(const sensor_msgs::Image::ConstPtr& img_msg)
   {
     imagedatas_kitti_.Push(img_msg);
   }

  void gpsKittiHandler(const sensor_msgs::NavSatFix::ConstPtr& gps)  //GPS信息
  {
    ROS_INFO("get kitti gps msg...");
    gpsdatas_kitti_.Push(gps);
    //std::cout<<gps->gps.longitude<<std::endl;
  }


  void ImuKittiHandler(const sensor_msgs::Imu::ConstPtr& imu)  //IMU信息
  {
    ROS_INFO("get kitti IMU msg...");
    imudatas_kitti_.Push(imu);
    //std::cout<<gps->gps.longitude<<std::endl;
  }


  void VelodyneKittiHandler(const sensor_msgs::PointCloud2::ConstPtr& hdl64)
  {
    ROS_INFO("get kitti hdl msg...");
    hdldatas_kitti_.Push(hdl64);
    if(hdldatas_kitti_.Size()>2)
      hdldatas_kitti_.Pop();
  }

  void process()
  {
    int frame_counter = 0;
    State_Vehicle ego_veh_state_gps;
    State_Vehicle ego_veh_state_lidar;
    while(!processthreadfinished_&&ros::ok())
    {
      timer t_total;
      //1)获取点云
      const sensor_msgs::PointCloud2ConstPtr cloudmsg = hdldatas_kitti_.PopWithTimeout(common::FromSeconds(0.1));
      if(cloudmsg == nullptr){
        ROS_WARN("No lidar data!");
        continue;
      }
      double lidartime = cloudmsg->header.stamp.toSec();
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_get(new pcl::PointCloud<pcl::PointXYZI>), tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
      pcl::fromROSMsg(*cloudmsg, *cloud_get);//获取当前帧点云数据
      //将kitti点云转换到本车常用车体坐标系下
      bool is_kitti_dataset = true;
      if (is_kitti_dataset)
        sensors_fusion::TransformKittiCloud(*cloud_get, *tempcloud, true, 1.73);
      else
        tempcloud = cloud_get;

      //2)获取相机图像
//      const sensor_msgs::ImageConstPtr image_msg = imagedatas_kitti_.Pop();
//      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg,  sensor_msgs::image_encodings::BGR8);
//      cv::Mat image_raw = cv_ptr->image;
//      imshow("image_raw", image_raw);
//      waitKey(5);
      //3)获取本车全局位姿
      sensor_msgs::NavSatFix::ConstPtr gpsdata;
      sensor_msgs::Imu::ConstPtr imudata;
      gpsdata = gpsdatas_kitti_.Pop();
      imudata = imudatas_kitti_.Pop();
      double heading = tf::getYaw(imudata->orientation);//航向,正东方向为0,逆时针[-180,180]

      memset(&ego_veh_state_gps, 0, sizeof(State_Vehicle));
      memset(&ego_veh_state_lidar, 0, sizeof(State_Vehicle));
      ego_veh_state_gps.is_update = true;
      ego_veh_state_gps.time_stamp = gpsdata->header.stamp.toSec();
      ego_veh_state_gps.fForwardVel = 0;
      ego_veh_state_gps.global_position.pitch = 0;//俯仰角
      ego_veh_state_gps.global_position.roll = 0;//侧倾角
      ego_veh_state_gps.global_position.heading = 90 - (heading*180/pi);//转为与正北方向的夹角

      if(ego_veh_state_gps.global_position.heading>=360)
        ego_veh_state_gps.global_position.heading = ego_veh_state_gps.global_position.heading - 360;
      if(ego_veh_state_gps.global_position.heading<0)
        ego_veh_state_gps.global_position.heading = ego_veh_state_gps.global_position.heading + 360;

      ego_veh_state_gps.global_position.dAld = 0;//gpsdata.;
      ego_veh_state_gps.global_position.dLat = gpsdata->latitude;
      ego_veh_state_gps.global_position.dLng = gpsdata->longitude;
      BaseFunction::Position_Trans_From_ECEF_To_UTM(gpsdata->latitude, gpsdata->longitude, 0,0,
          &ego_veh_state_gps.global_position.dLng, &ego_veh_state_gps.global_position.dLat);

      ++frame_counter;
      std::cout<<"========================= Begin Process "<<frame_counter<<" ============================"<<std::endl;

      State_Vehicle ego_veh_state_current = ego_veh_state_gps;//最终使用的就是这个
      if(tempcloud->points.size() && ego_veh_state_current.is_update)
      {
        ROS_WARN("Input cloud size is %d",tempcloud->points.size());
        moving_target.SetBasicInput(lidartime,ego_veh_state_current);
        moving_target.SetCloudInput(tempcloud);//拿到点云
        //开始处理
        moving_target.ProcessFrame();
        moving_target.DrawResult();
        //可视化
        moving_target.Show_Result_2D(t_total.elapsed());

        //发布特定点云话题
        MovingTargetTrack::CloudPtr elevated_cloud_ptr = moving_target.get_elevated_cloud_ptr();
        elevated_cloud_ptr->header.frame_id = "/vehicle_link";
        pub_elevated_cloud_.publish(*elevated_cloud_ptr);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud_ptr = moving_target.getRGBClusteredCloud();
        clustered_cloud_ptr->header.frame_id = "/vehicle_link";
        pub_clustered_rgb_cloud_.publish(*clustered_cloud_ptr);

//        cv::Mat img_clustered = sensors_fusion::ProjectRGBCloud2Image(clustered_cloud_ptr, image_raw, this->project_matrix_);
//        namedWindow("img_fusion_result", CV_WINDOW_NORMAL);
//        imshow("img_fusion_result", img_clustered);
      }

      std::cout<< " t_total: " << t_total.elapsed() << endl;
      std::cout<<"========================= End Process==============================="<<std::endl<<std::endl<<std::endl;
    }//end while(!processthreadfinished_)
  }

protected:
  ros::NodeHandle& nodehandle_;
  //ROS subscriber
  ros::Subscriber subGpsKitti_;//GPS-for vehicle global position
  ros::Subscriber subImu_; //订阅IMU信息 - get vehicle pose
  ros::Subscriber subVelodyne_; //订阅Kitti 64线激光雷达数据
  ros::Subscriber subImage_; //color image
  image_transport::CameraSubscriber subCamera_;
  //ROS publisher
  ros::Publisher pub_target_send;
  ros::Publisher pub_elevated_cloud_;//removed ground cloud
  ros::Publisher pub_clustered_rgb_cloud_;

  common::BlockingQueue<sensor_msgs::Imu::ConstPtr> imudatas_kitti_;
  common::BlockingQueue<sensor_msgs::PointCloud2::ConstPtr> hdldatas_kitti_;
  common::BlockingQueue<sensor_msgs::NavSatFix::ConstPtr> gpsdatas_kitti_;
  common::BlockingQueue<sensor_msgs::Image::ConstPtr> imagedatas_kitti_;

  boost::thread* processthread_;
  bool processthreadfinished_;
  Eigen::MatrixXf project_matrix_;//transformation matrix used for project lidar point to camera image, should be 3x4

  MovingTargetTrack moving_target;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamicobject_kitti");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);
  ros::spin();
  return 0;
}
