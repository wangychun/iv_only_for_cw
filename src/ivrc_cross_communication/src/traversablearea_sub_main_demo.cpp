#include <ros/ros.h>
#include "iv_slam_ros_msgs/TraversableArea.h"
#include "sensor_msgs/Image.h"
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/Odometry.h"
#include "iomanip"
#include "rigid_transform.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "transform.h"




nav_msgs::Odometry received_tem_global_vehicle_pose;
iv_slam_ros_msgs::TraversableArea TwiDTraversableArea;
void vehicle_pose_topic_callback(const nav_msgs::Odometry &sub_tem_global_vehicle_pose){
  received_tem_global_vehicle_pose.header.stamp = sub_tem_global_vehicle_pose.header.stamp;
  received_tem_global_vehicle_pose.header.frame_id = sub_tem_global_vehicle_pose.header.frame_id;
  received_tem_global_vehicle_pose.pose.pose.position.x = sub_tem_global_vehicle_pose.pose.pose.position.x;//longitude of the current UGV positon
  received_tem_global_vehicle_pose.pose.pose.position.y = sub_tem_global_vehicle_pose.pose.pose.position.y;//latitude of the current UGV positon
  received_tem_global_vehicle_pose.pose.pose.position.z = sub_tem_global_vehicle_pose.pose.pose.position.z;// the following result is pose relative to the start point of UGV
  received_tem_global_vehicle_pose.pose.pose.orientation.w = sub_tem_global_vehicle_pose.pose.pose.orientation.w;
  received_tem_global_vehicle_pose.pose.pose.orientation.x = sub_tem_global_vehicle_pose.pose.pose.orientation.x;
  received_tem_global_vehicle_pose.pose.pose.orientation.y = sub_tem_global_vehicle_pose.pose.pose.orientation.y;
  received_tem_global_vehicle_pose.pose.pose.orientation.z = sub_tem_global_vehicle_pose.pose.pose.orientation.z;

}
void testcallback(const iv_slam_ros_msgs::TraversableArea &subed_msg){

  TwiDTraversableArea.header.stamp = subed_msg.header.stamp;
  TwiDTraversableArea.header.frame_id = subed_msg.header.frame_id;//which is "traversible_area_frame"
  TwiDTraversableArea.resolution = subed_msg.resolution; //resolution of the submap 
  TwiDTraversableArea.submap_finished_flag = subed_msg.submap_finished_flag;//flag to decide whether the current submap is finished;
  TwiDTraversableArea.triD_submap_pose_image_index_x = subed_msg.triD_submap_pose_image_index_x; // location in the pictrue of the submap pose.
  TwiDTraversableArea.triD_submap_pose_image_index_y = subed_msg.triD_submap_pose_image_index_y;

  TwiDTraversableArea.triD_submap_pose.position.x = subed_msg.triD_submap_pose.position.x;//longitude of the submap
  TwiDTraversableArea.triD_submap_pose.position.y = subed_msg.triD_submap_pose.position.y;//latitude of the submap
  TwiDTraversableArea.triD_submap_pose.position.z = subed_msg.triD_submap_pose.position.z;// position relative to the start point of UGV
  TwiDTraversableArea.triD_submap_pose.orientation.w = subed_msg.triD_submap_pose.orientation.w;//the following is the pose in the world frame
  TwiDTraversableArea.triD_submap_pose.orientation.x = subed_msg.triD_submap_pose.orientation.x;
  TwiDTraversableArea.triD_submap_pose.orientation.y = subed_msg.triD_submap_pose.orientation.y;
  TwiDTraversableArea.triD_submap_pose.orientation.z = subed_msg.triD_submap_pose.orientation.z;

  TwiDTraversableArea.width = subed_msg.width;
  TwiDTraversableArea.height = subed_msg.height;  
  TwiDTraversableArea.cells.assign(subed_msg.cells.begin(),subed_msg.cells.end());  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversablearea_sub_main_demo");
  ros::NodeHandle nh; 
  ros::Subscriber test = nh.subscribe("traversible_area_topic",1,testcallback);
  ros::Subscriber vehicle_global_pose_topic_sub = nh.subscribe("vehicle_global_pose_topic",1,vehicle_pose_topic_callback);
  double a=6378137;
  double e2= 0.0818192*0.0818192;//e的平方
  cartographer::transform::GridZone zone =cartographer::transform:: UTM_ZONE_51;
  cartographer::transform::Hemisphere hemi = cartographer::transform:: HEMI_NORTH;

  ros::Rate rate(100);

  while(ros::ok())
  {
    if(TwiDTraversableArea.width==0&& TwiDTraversableArea.height==0){      

      ros::spinOnce();
      continue;

    }
    //     std::cout<<std::fixed<<std::setprecision(10)<< received_tem_global_vehicle_pose.pose.pose.position <<std::endl;
    //     std::cout<< received_tem_global_vehicle_pose.pose.pose.orientation <<std::endl;
    //     std::cout<<std::fixed<<std::setprecision(10)<<  TwiDTraversableArea.triD_submap_pose.position <<std::endl;
    //     std::cout<< TwiDTraversableArea.triD_submap_pose.orientation <<std::endl;

    cartographer::transform:: geographic_to_grid( a, e2, (TwiDTraversableArea.triD_submap_pose.position.y)*M_PI/180, (TwiDTraversableArea.triD_submap_pose.position.x)*M_PI/180,
        &zone, &hemi,&(TwiDTraversableArea.triD_submap_pose.position.y), &(TwiDTraversableArea.triD_submap_pose.position.x));
    cartographer::transform::Rigid3d TwiDTraversableArea_pose = cartographer::transform::Rigid3d(
        Eigen::Vector3d(TwiDTraversableArea.triD_submap_pose.position.x ,TwiDTraversableArea.triD_submap_pose.position.y,TwiDTraversableArea.triD_submap_pose.position.z),
        Eigen::Quaternion<double>( TwiDTraversableArea.triD_submap_pose.orientation.w, TwiDTraversableArea.triD_submap_pose.orientation.x,
            TwiDTraversableArea.triD_submap_pose.orientation.y, TwiDTraversableArea.triD_submap_pose.orientation.z
        ));
    std::cout<<"TwiDTraversableArea_pose:"<<TwiDTraversableArea<<std::endl;
    cartographer::transform:: geographic_to_grid( a, e2, (received_tem_global_vehicle_pose.pose.pose.position.y)*M_PI/180, (received_tem_global_vehicle_pose.pose.pose.position.x)*M_PI/180,
        &zone, &hemi,&(received_tem_global_vehicle_pose.pose.pose.position.y), &(received_tem_global_vehicle_pose.pose.pose.position.x));

    cartographer::transform::Rigid3d  global_vehicle_pose = cartographer::transform::Rigid3d(Eigen::Vector3d(received_tem_global_vehicle_pose.pose.pose.position.x,
        received_tem_global_vehicle_pose.pose.pose.position.y,received_tem_global_vehicle_pose.pose.pose.position.z
    ),Eigen::Quaternion<double>(received_tem_global_vehicle_pose.pose.pose.orientation.w,received_tem_global_vehicle_pose.pose.pose.orientation.x,
        received_tem_global_vehicle_pose.pose.pose.orientation.y,received_tem_global_vehicle_pose.pose.pose.orientation.z
    ));

    Eigen::Vector3d submap_eular_pose  = cartographer::transform::QuaterniondtoPitchRollYaw(TwiDTraversableArea_pose.rotation());
    Eigen::Vector3d global_vehicle_eular_pose = cartographer::transform::QuaterniondtoPitchRollYaw(global_vehicle_pose.rotation());
    global_vehicle_pose = cartographer::transform::Rigid3d(global_vehicle_pose.translation(),
        cartographer::transform::PitchRollYaw(submap_eular_pose.x(),submap_eular_pose.y(),global_vehicle_eular_pose.z()));

    IplImage * testimage = cvCreateImage(cvSize( TwiDTraversableArea.width, TwiDTraversableArea.height),IPL_DEPTH_8U,3);
    IplImage * showimage = cvCreateImage(cvSize( 400, 800),IPL_DEPTH_8U,3);
    int data_index = 0;
    cvZero(testimage);
    cvZero(showimage);

    int x_index2,y_index2;
    unsigned char * pdata;
    //     double start_time = ros::Time::now().toSec();
    for(int i = 0;i<testimage->height;i++){
      for(int j = 0; j<testimage->width;j++){
        if((TwiDTraversableArea.cells.at(data_index)==2)){

          cartographer::transform::Rigid3d tem_pose(Eigen::Vector3d((
              ( data_index%testimage->width-TwiDTraversableArea.triD_submap_pose_image_index_x)*TwiDTraversableArea.resolution),
              (
                  ( data_index/testimage->width-TwiDTraversableArea.triD_submap_pose_image_index_y)*TwiDTraversableArea.resolution),
                  0),Eigen::Quaternion<double>(1,0,0,0
                  ));

          std::cout<<"TwiDTraversableArea_pose:"<<TwiDTraversableArea_pose<<std::endl;
          std::cout<<"tem_pose:"<<tem_pose<<std::endl;
          cartographer::transform::Rigid3d global_pose = TwiDTraversableArea_pose *tem_pose;

          x_index2 = (global_vehicle_pose.inverse()* global_pose).translation().x()/TwiDTraversableArea.resolution;
          y_index2 = (global_vehicle_pose.inverse()* global_pose).translation().y()/TwiDTraversableArea.resolution;

          if(x_index2>-200&&x_index2<200&&y_index2>-400&&y_index2<400){

            pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-y_index2-400)*showimage->widthStep +(x_index2+200)*3 ) ;
            pdata[0] = 254;
            pdata[1] = 254;
            pdata[2] = 254;
          }

        }else if(TwiDTraversableArea.cells.at(data_index)==1){

          cartographer::transform::Rigid3d tem_pose(Eigen::Vector3d((
              ( data_index%testimage->width-TwiDTraversableArea.triD_submap_pose_image_index_x)*TwiDTraversableArea.resolution),
              (
                  ( data_index/testimage->width-TwiDTraversableArea.triD_submap_pose_image_index_y)*TwiDTraversableArea.resolution),
                  0),Eigen::Quaternion<double>(1,0,0,0
                  ));


          cartographer::transform::Rigid3d global_pose = TwiDTraversableArea_pose *tem_pose;

          x_index2 = (global_vehicle_pose.inverse()* global_pose).translation().x()/TwiDTraversableArea.resolution;
          y_index2 = (global_vehicle_pose.inverse()* global_pose).translation().y()/TwiDTraversableArea.resolution;

          if(x_index2>-200&&x_index2<200&&y_index2>-400&&y_index2<400){

            pdata = (unsigned char *)(showimage->imageData+(showimage->height-1-y_index2-400)*showimage->widthStep +(x_index2+200)*3 ) ;
            pdata[0] = 0;
            pdata[1] = 0;
            pdata[2] = 0;
          }
        }
        data_index++;
      }}

    //         double stop_time = ros::Time::now().toSec();

    // 	std::cout<<"!!!!!!!!!!!!!!!1stop_time -start_time"<<stop_time -start_time<<std::endl;

    cvCircle(showimage,cvPoint(200, 400),5, cvScalar(255,0,0),-1);

    cvShowImage("traversible area puslished",showimage);
    cvWaitKey(5);
    cvReleaseImage(&testimage);
    cvReleaseImage(&showimage);

    ros::spinOnce();
    rate.sleep();
  }
}
