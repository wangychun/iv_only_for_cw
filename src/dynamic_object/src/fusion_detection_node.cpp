/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2018年12月1日
* Copyright    :
* Descriptoin  :
* References   :
======================================================================*/

#include <ros/ros.h>
#include <sensors_fusion/fusion_detection.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "fusion_detection");
    sensors_fusion::FusionDetection fusion_detection(
        ros::NodeHandle(), ros::NodeHandle("~"));
    ros::spin();

    return 0;
}


