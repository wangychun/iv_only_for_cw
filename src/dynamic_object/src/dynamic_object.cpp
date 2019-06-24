#include "transform/rigid_transform.h"
#include "tracker/imm_ukf_jpda.h"
#include <common/common.h>
#include <nav_msgs/Odometry.h>
//C++
#include <cmath>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <list>
#include <glog/logging.h>

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/mouse_event.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>   //
#include <pcl/octree/octree.h>

//Boost
#include <boost/timer.hpp>

#include "StructMovingTargetDefine.h"
#include "multi_object_tracking.h"
#include "util/boostudp/boostudp.h"
#include "common/blocking_queue.h"
#include "common/make_unique.h"
#include "src/velodyne/data_types.hpp"
#include "sensor_driver_msgs/OdometrywithGps.h"
#include "sensor_driver_msgs/GpswithHeading.h"
//#include "sensor_driver_msgs/InsVelocity.h"
#include "iv_dynamicobject_msgs/moving_target.h"
#include "iv_dynamicobject_msgs/moving_target_send.h"
#include "iv_dynamicobject_msgs/Predict_traj.h"
#include "iv_dynamicobject_msgs/History_traj.h"
#include "iv_dynamicobject_msgs/Points.h"
#include "iv_dynamicobject_msgs/Rectangle.h"
#include "iv_dynamicobject_msgs/TargetCar.h"
#include "iv_dynamicobject_msgs/RadarPoint.h"
#include "iv_dynamicobject_msgs/RadarData.h"
#include "iv_dynamicobject_msgs/RadarData.h"
#include <iomanip>
#include <visualization_msgs/Marker.h>

#define LOCAL_IP "192.168.0.112"
//#define LOCAL_IP "127.0.0.1"
#define FROMLADAR_LOCAL_PORT 9906

class PostProcess
{
public:
    typedef std::pair<double,transform::Rigid3d> TimePosePair;
    PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle)
    ,processthread_(NULL)
    ,processthreadfinished_ (false)
    {
        init();
    }
    ~PostProcess()
    {
        processthreadfinished_ = true;
        processthread_->join();
    }

    void init()
    {

        subLaserOdometry_ = nodehandle_.subscribe<sensor_driver_msgs::OdometrywithGps>
                ("lidar_odometry_to_earth", 5, boost::bind(&PostProcess::laserOdometryHandler,this,_1));//需要雷达里程计信息时需要，否则可以注释掉
        subLaserCloudFullRes_ = nodehandle_.subscribe<sensor_msgs::PointCloud2>
                ("lidar_cloud_calibrated", 1, boost::bind(&PostProcess::laserCloudHandler,this,_1));//经过筛选且转换之后的点云
        subGps_ = nodehandle_.subscribe<sensor_driver_msgs::GpswithHeading>
                ("gpsdata", 1, boost::bind(&PostProcess::gpsHandler,this,_1));//经过筛选且转换之后的点云

        subRadar_ = nodehandle_.subscribe<iv_dynamicobject_msgs::RadarData>
                ("radardata", 1, boost::bind(&PostProcess::radarHandler,this,_1));//经过筛选且转换之后的点云
        //subInsVel_ = nodehandle_.subscribe<sensor_driver_msgs::InsVelocity>
               // ("insvelocity", 1, boost::bind(&PostProcess::insvleHandler,this,_1));//


        processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));

        pub_target_send = nodehandle_.advertise<iv_dynamicobject_msgs::moving_target_send>("MovingTarget",1);

        pub_show3D = nodehandle_.advertise<iv_dynamicobject_msgs::TargetCar>("Result",1);
        pubCloud_ = nodehandle_.advertise<sensor_msgs::PointCloud2> ("output", 1);
        vis_pub_ = nodehandle_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

        //file_.open("/home/jkj/catkin_ws/result.txt",std::ios::out);
    }


//    void insvleHandler(const sensor_driver_msgs::InsVelocity::ConstPtr& ins_vel_msg_)
//    {
//        ins_vel_data_.Push(ins_vel_msg_);
//    }
    void radarHandler(const iv_dynamicobject_msgs::RadarData::ConstPtr& radar_msg_)
    {
        radardatas_.Push(radar_msg_);
    }
    void gpsHandler(const sensor_driver_msgs::GpswithHeading::ConstPtr& gps)  //雷达里程计
    {
//      ROS_INFO("<dynamicobject> GPS data callback...");
        gpsdatas_.Push(gps);
        //std::cout<<gps->gps.longitude<<std::endl;
    }

    void laserOdometryHandler(const sensor_driver_msgs::OdometrywithGps::ConstPtr& laserOdometry)
    {
      ROS_INFO("<dynamicobject> laserOdometryHandler data callback...");
        double timeOdometry = laserOdometry->odometry.header.stamp.toSec();
        static double last_stamp = -1;
        //  static geometry_msgs::Quaternion last_geoQuat;
        static transform::Rigid3d lasttransformodometry;
        //  static float last_trans[6];
        //  double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->odometry.pose.pose.orientation;

        Eigen::Quaterniond roatation(geoQuat.w,geoQuat.x,geoQuat.y,geoQuat.z);
        Eigen::Vector3d translation(laserOdometry->odometry.pose.pose.position.x,
                                    laserOdometry->odometry.pose.pose.position.y,
                                    laserOdometry->odometry.pose.pose.position.z);

        transform::Rigid3d transformodometry(translation,roatation);
        lidarOdoms_.Push(common::make_unique<TimePosePair>(timeOdometry,transformodometry));
    }

    void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullRes2) //点云数据
    {
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

    void analysisRadar(const iv_dynamicobject_msgs::RadarData::ConstPtr radar_msg_, Radar_data* radar_)
    {
        radar_->ACC_Target_ID = radar_msg_->ACC_Target_ID;
        radar_->target_num = 0;
        for (int i = 0; i < 64; ++i)
        {
            iv_dynamicobject_msgs::RadarPoint temp_radar_point = radar_msg_->delphi_detection_array[i];
            {
                Radar_Target temp_target;
                temp_target.time_stamp = radar_->time_stamp;
                temp_target.range = temp_radar_point.range;
                temp_target.angle = temp_radar_point.angle;
                temp_target.x = temp_radar_point.x;
                temp_target.y = temp_radar_point.y+4.0;
                temp_target.v = temp_radar_point.v;
                temp_target.v_x = temp_target.v*cos(temp_target.angle);
                temp_target.v_y = temp_target.v*sin(temp_target.angle);
                temp_target.moving = temp_radar_point.moving;
                temp_target.moving_fast = temp_radar_point.moving_fast;
                temp_target.moving_slow = temp_radar_point.moving_slow;
                temp_target.status = temp_radar_point.status;
                temp_target.valid = temp_radar_point.valid;
                temp_target.target_ID = temp_radar_point.target_ID;
                temp_target.match_index = -1;

                radar_->target[i] = temp_target;
                radar_->target_num++;
            }
        }
    }

    void Moving_target_out(iv_dynamicobject_msgs::moving_target* target_send_,  Target_output2 target_output_)
    {
        target_send_->line_num = target_output_.line_num;
        target_send_->line_point.clear();
        for (int i = 0; i < target_output_.line_num; ++i)
        {
            iv_dynamicobject_msgs::Points temp_point;
            temp_point.x = target_output_.line_point[i].x;
            temp_point.y = target_output_.line_point[i].y;
            temp_point.z = 0;
            target_send_->line_point.push_back(temp_point);
        }
        target_send_->center_point.x = target_output_.center_point.x;
        target_send_->center_point.y = target_output_.center_point.y;
        target_send_->center_point.z = 0;
        target_send_->object_high = target_output_.object_high;
        target_send_->object_type = target_output_.object_type;
        target_send_->ID_number = target_output_.ID_number;
        target_send_->is_updated = target_output_.is_updated;
        target_send_->tracked_times = target_output_.tracked_times;
        target_send_->dangerous_level = target_output_.dangerous_level;
        target_send_->predict_num = target_output_.predict_num;
        target_send_->predict_traj.clear();
        for (int i = 0; i < target_send_->predict_num; ++i)
        {
            iv_dynamicobject_msgs::Predict_traj temp_traj;
            temp_traj.time_stamp = target_output_.predict_traj[i].time_stamp;
            temp_traj.point.x = target_output_.predict_traj[i].point.x;
            temp_traj.point.y = target_output_.predict_traj[i].point.y;
            temp_traj.point.z = target_output_.predict_traj[i].time_stamp;
            temp_traj.v_x = target_output_.predict_traj[i].v_x;
            temp_traj.v_y = target_output_.predict_traj[i].v_y;
            temp_traj.acc_x = target_output_.predict_traj[i].acc_x;
            temp_traj.acc_y = target_output_.predict_traj[i].acc_y;
            temp_traj.pos_head = target_output_.predict_traj[i].pos_head;
            temp_traj.v_w = target_output_.predict_traj[i].v_w;
            temp_traj.confidence_level = target_output_.predict_traj[i].confidence_level;

            target_send_->predict_traj.push_back(temp_traj);
        }
        target_send_->history_num = target_output_.history_num;
        target_send_->history_traj.clear();
        for (int i = 0; i < target_send_->history_num; ++i)
        {
            iv_dynamicobject_msgs::History_traj temp_traj;
            temp_traj.time_stamp = target_output_.history_traj[i].history_time;
            temp_traj.center_point.x = target_output_.history_traj[i].history_center.x;
            temp_traj.center_point.y = target_output_.history_traj[i].history_center.y;
            temp_traj.center_point.z = 0;
            temp_traj.line_num = 4;
            temp_traj.line_point.clear();
            for (int j = 0; j < temp_traj.line_num; ++j)
            {
                iv_dynamicobject_msgs::Points temp_point;
                temp_point.x = target_output_.history_traj->history_rect.point4[j].x;
                temp_point.y = target_output_.history_traj->history_rect.point4[j].y;
                temp_point.z = 0;
                temp_traj.line_point.push_back(temp_point);
            }
            target_send_->history_traj.push_back(temp_traj);
        }
    }

    void SetViewerPCL(boost::shared_ptr<PCLVisualizer> cloud_viewer_)
    {
        cloud_viewer_->addCoordinateSystem (1.0);
        cloud_viewer_->setBackgroundColor(0,0,0);//(0.75,0.75,0.75);//(1.0,1.0,1.0);// (255, 0, 0);
        cloud_viewer_->initCameraParameters();
        cloud_viewer_->setCameraPosition(0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0);
        cloud_viewer_->setCameraClipDistances(1.0, 120.0);
    }


    void process()
    {
        frame_counter = 0;

        outputcloud_vector.clear();
        lidartime_vector.clear();

        while(!processthreadfinished_&&ros::ok())
        {
            frame_counter++;
            const sensor_msgs::PointCloud2ConstPtr cloudmsg = lidarCloudMsgs_.PopWithTimeout(common::FromSeconds(0.1));
            if(cloudmsg == nullptr)
                continue;
            double lidartime = cloudmsg->header.stamp.toSec();

            pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);//当前帧点云（雷达里程计坐标系）
            pcl::fromROSMsg(*cloudmsg, *tempcloud);//获取当前帧点云数据
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputclouds;
            std::vector<pcl::PointXYZI> lidarpropertys;
            analysisCloud(tempcloud, outputclouds, lidarpropertys);

            if(tempcloud->points.size())
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
              pcl::copyPointCloud(*tempcloud,*cloud_in);
              multi_tracker.SetCloudInput(*cloud_in);
              multi_tracker.ProcessFrame();
              printf("ProcessFrame...%d\n ",cloud_in->size());

              pcl::PointCloud<pcl::PointXYZ>::Ptr elevated_cloud = multi_tracker.getElevatedCloud();
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_Cloud = multi_tracker.getClusteredCloud();
              elevated_cloud->header.frame_id = "velo_link"; // add "velo_link"
              clustered_Cloud->header.frame_id = "velo_link"; // add "velo_link"
              //可视化
              sensor_msgs::PointCloud2 output;
              toROSMsg(*clustered_Cloud, output);
              tempcloud->header.frame_id = "velo_link";
//              toROSMsg(*elevated_cloud, output);
              printf("elevated_cloud...%d\n ",clustered_Cloud->size());
              // Publish the data.
              pubCloud_.publish(output);

              std::vector<pcl::PointCloud<pcl::PointXYZ>> visBBs = multi_tracker.getBoxes();
              for(int i = 0;i < visBBs.size();++i)
              {
                visBBs[i].header.frame_id = "velo_link";
              }

              /*****************************
               * Main tracking handlers
               *****************************/
              pcl::PointCloud<pcl::PointXYZ> targetPoints;
              vector<vector<double>> targetVandYaw;
              vector<int> trackManage;
              vector<bool> isStaticVec;
              vector<bool> isVisVec;
//              std::vector<pcl::PointCloud<pcl::PointXYZ>> visBBs;
//              //输入:全局坐标系下的bBoxes, 原始雷达数据获取时时间戳timestamp
//              //输出:都是全局坐标系下.目标中心点targetPoints,目标运动速度和方位角targetVandYaw,目标跟踪管理状态trackManage
//              //    目标是否静止isStaticVec,是否可视化运动目标运动方向isVisVec,目标包围框visBBs
//              immUkfJpdaf(bBoxes, lidartime, targetPoints, targetVandYaw, trackManage, isStaticVec, isVisVec, visBBs);
//

              // tracking arrows visualizing start---------------------------------------------
              for(int i = 0; i < targetPoints.size(); i++){
                visualization_msgs::Marker arrowsG;
                arrowsG.lifetime = ros::Duration(0.1);
                if(trackManage[i] == 0 ) {
                  continue;
                }
                if(isVisVec[i] == false ) {
                  continue;
                }
                if(isStaticVec[i] == true){
                  continue;
                }
                arrowsG.header.frame_id = "/velo_link";
                arrowsG.header.stamp= ros::Time::now();
                arrowsG.ns = "arrows";
                arrowsG.action = visualization_msgs::Marker::ADD;
                arrowsG.type =  visualization_msgs::Marker::ARROW;
                // green
                arrowsG.color.g = 1.0f;
                arrowsG.color.a = 1.0;
                arrowsG.id = i;
                geometry_msgs::Point p;
                // assert(targetPoints[i].size()==4);
                p.x = targetPoints[i].x;
                p.y = targetPoints[i].y;
                p.z = 1.73/2;
                double tv   = targetVandYaw[i][0];
                double tyaw = targetVandYaw[i][1];

                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                arrowsG.pose.position.x = p.x;
                arrowsG.pose.position.y = p.y;
                arrowsG.pose.position.z = p.z;

                // convert from 3 angles to quartenion
                tf::Matrix3x3 obs_mat;
                obs_mat.setEulerYPR(tyaw, 0, 0); // yaw, pitch, roll
                tf::Quaternion q_tf;
                obs_mat.getRotation(q_tf);
                arrowsG.pose.orientation.x = q_tf.getX();
                arrowsG.pose.orientation.y = q_tf.getY();
                arrowsG.pose.orientation.z = q_tf.getZ();
                arrowsG.pose.orientation.w = q_tf.getW();

                // Set the scale of the arrowsG -- 1x1x1 here means 1m on a side
                arrowsG.scale.x = tv;
                arrowsG.scale.y = 0.1;
                arrowsG.scale.z = 0.1;

                vis_pub_.publish(arrowsG);
              }
              // tracking arrows visualizing end---------------------------------------------

              // tracking points visualizing start---------------------------------------------
              visualization_msgs::Marker pointsY, pointsG, pointsR, pointsB;
              pointsY.header.frame_id = pointsG.header.frame_id = pointsR.header.frame_id = pointsB.header.frame_id = "velo_link";
              pointsY.header.stamp= pointsG.header.stamp= pointsR.header.stamp =pointsB.header.stamp = ros::Time::now();
              pointsY.ns= pointsG.ns = pointsR.ns =pointsB.ns=  "points";
              pointsY.action = pointsG.action = pointsR.action = pointsB.action = visualization_msgs::Marker::ADD;
              pointsY.pose.orientation.w = pointsG.pose.orientation.w  = pointsR.pose.orientation.w =pointsB.pose.orientation.w= 1.0;

              pointsY.id = 1;
              pointsG.id = 2;
              pointsR.id = 3;
              pointsB.id = 4;
              pointsY.type = pointsG.type = pointsR.type = pointsB.type = visualization_msgs::Marker::POINTS;

              // POINTS markers use x and y scale for width/height respectively
              pointsY.scale.x =pointsG.scale.x =pointsR.scale.x = pointsB.scale.x=0.5;
              pointsY.scale.y =pointsG.scale.y =pointsR.scale.y = pointsB.scale.y = 0.5;

              // yellow
              pointsY.color.r = 1.0f;
              pointsY.color.g = 1.0f;
              pointsY.color.b = 0.0f;
              pointsY.color.a = 1.0;

              // green
              pointsG.color.g = 1.0f;
              pointsG.color.a = 1.0;

              // red
              pointsR.color.r = 1.0;
              pointsR.color.a = 1.0;

              // blue
              pointsB.color.b = 1.0;
              pointsB.color.a = 1.0;

              for(int i = 0; i < targetPoints.size(); i++){
                if(trackManage[i] == 0) continue;
                geometry_msgs::Point p;
                // p.x = targetPoints[i].x;
                // p.y = targetPoints[i].y;
                p.x = targetPoints[i].x;
                p.y = targetPoints[i].y;
                p.z = -1.73/2;
                // cout << trackManage[i] << endl;
                if(isStaticVec[i] == true){
                  pointsB.points.push_back(p);
                }
                else if(trackManage[i] < 5 ){
                  pointsY.points.push_back(p);
                }
                else if(trackManage[i] == 5){
                  pointsG.points.push_back(p);
                }
                else if(trackManage[i] > 5){
                  pointsR.points.push_back(p);
                }
              }
              vis_pub_.publish(pointsY);
              // cout << "pointsG" << pointsG.points[0].x << " "<< pointsG.points[0].y << endl;
              vis_pub_.publish(pointsG);
              vis_pub_.publish(pointsR);
              vis_pub_.publish(pointsB);
              // tracking poiints visualizing end---------------------------------------------

              // bounding box visualizing start---------------------------------------------
              visualization_msgs::Marker line_list;
              line_list.header.frame_id = "velo_link";
              line_list.header.stamp = ros::Time::now();
              line_list.ns =  "boxes";
              line_list.action = visualization_msgs::Marker::ADD;
              line_list.pose.orientation.w = 1.0;

              line_list.id = 0;
              line_list.type = visualization_msgs::Marker::LINE_LIST;

              //LINE_LIST markers use only the x component of scale, for the line width
              line_list.scale.x = 0.1;
              // Points are green
              line_list.color.g = 1.0f;
              line_list.color.a = 1.0;

              int id = 0;string ids;
              for(int objectI = 0; objectI < visBBs.size(); objectI ++){
                for(int pointI = 0; pointI < 4; pointI++){
                  assert((pointI+1)%4 < visBBs[objectI].size());
                  assert((pointI+4) < visBBs[objectI].size());
                  assert((pointI+1)%4+4 < visBBs[objectI].size());
                  id ++; ids = to_string(id);
                  geometry_msgs::Point p;
                  p.x = visBBs[objectI][pointI].x;
                  p.y = visBBs[objectI][pointI].y;
                  p.z = visBBs[objectI][pointI].z;
                  line_list.points.push_back(p);
                  p.x = visBBs[objectI][(pointI+1)%4].x;
                  p.y = visBBs[objectI][(pointI+1)%4].y;
                  p.z = visBBs[objectI][(pointI+1)%4].z;
                  line_list.points.push_back(p);

                  p.x = visBBs[objectI][pointI].x;
                  p.y = visBBs[objectI][pointI].y;
                  p.z = visBBs[objectI][pointI].z;
                  line_list.points.push_back(p);
                  p.x = visBBs[objectI][pointI+4].x;
                  p.y = visBBs[objectI][pointI+4].y;
                  p.z = visBBs[objectI][pointI+4].z;
                  line_list.points.push_back(p);

                  p.x = visBBs[objectI][pointI+4].x;
                  p.y = visBBs[objectI][pointI+4].y;
                  p.z = visBBs[objectI][pointI+4].z;
                  line_list.points.push_back(p);
                  p.x = visBBs[objectI][(pointI+1)%4+4].x;
                  p.y = visBBs[objectI][(pointI+1)%4+4].y;
                  p.z = visBBs[objectI][(pointI+1)%4+4].z;
                  line_list.points.push_back(p);
                }
              }

              //line list end
              vis_pub_.publish(line_list);
              // bounding box visualizing end---------------------------------------------


            }

        }//end while(!processthreadfinished_)
    }

protected:

    ros::Subscriber subLaserCloudFullRes_ ;//经过筛选且转换之后的点云
    ros::Subscriber subLaserOdometry_ ;
    ros::Subscriber subGps_ ;//经过筛选且#include "sensor_driver_msgs/moving_target.h"转换之后的点云
    ros::Subscriber subRadar_ ;
    ros::Subscriber subInsVel_ ;

    ros::Publisher pub_target_send;
    ros::Publisher pub_show3D;
    ros::Publisher pubCloud_,vis_pub_;

    common::BlockingQueue<sensor_msgs::PointCloud2ConstPtr> lidarCloudMsgs_;
    common::BlockingQueue<std::unique_ptr<TimePosePair>> lidarOdoms_;
    common::BlockingQueue<sensor_driver_msgs::GpswithHeading::ConstPtr> gpsdatas_;
    //common::BlockingQueue<sensor_driver_msgs::InsVelocity::ConstPtr> ins_vel_data_;
    common::BlockingQueue<iv_dynamicobject_msgs::RadarData::ConstPtr> radardatas_;
//  std::fstream file_;
    ros::NodeHandle& nodehandle_;
    boost::thread* processthread_;
    bool processthreadfinished_;


    State_Vehicle ego_veh_state_current;
    State_Vehicle ego_veh_state_lidar;
    State_Vehicle ego_veh_state_gps;
    MultiObjectTracking multi_tracker;

    std::vector<double> lidartime_vector;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> outputcloud_vector;
    int frame_counter;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamic_object");
  ros::NodeHandle nh;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  FLAGS_colorlogtostderr = true; //设置输出到屏幕的日志显示相应颜色
  PostProcess postprocess(nh);
  ros::spin();


  return 0;
}
