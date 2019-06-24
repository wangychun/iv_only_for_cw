#include <ros/ros.h>
#include "iv_slam_ros_msgs/TraversableArea.h"
#include "sensor_msgs/Image.h"
#include <octomap/octomap.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "nav_msgs/Odometry.h"



int main(int argc, char** argv)
{
  ros::init(argc, argv, "traversablearea_sub_main");
  ros::NodeHandle nh;  
// ros::Publisher test_pub = nh.advertise();
 
 ros::Rate rate(10);

  while(ros::ok())
  {

     ros::spinOnce();
     rate.sleep();
  }
}
