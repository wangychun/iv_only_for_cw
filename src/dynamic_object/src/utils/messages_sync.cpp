/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年11月24日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include "messages_sync.h"
//C++
#include <iomanip>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
//#include "boost/asio.hpp"
namespace sensors_fusion {

////////////////////////////////MessagesSync////////////////////////////////////////////////////////
MessagesSync::MessagesSync(ros::NodeHandle nh, std::string camera_topic_name, std::string lidar_topic_name):
    nodeHandle_(nh),
    subCameraImage_(nh, camera_topic_name, 2),
    subLidarData_(nh,lidar_topic_name, 2),
    flag(false),
    sync(MySyncPolicy(10), subCameraImage_, subLidarData_)
{
  //发布同步后的数据
  ROS_INFO("lidar and camera data synchronizing started");
  sync.registerCallback(boost::bind(&MessagesSync::cameraLidarCallback, this,_1, _2));
}

MessagesSync::SyncImageCloudPair MessagesSync::getSyncMessages()
{
  if(!flag)
    return SyncImageCloudPair();
  else {
    flag = false;
    return syncMessages_;
  }
}

void MessagesSync::cameraLidarCallback(const sensor_msgs::ImageConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
  ROS_INFO_THROTTLE(5, "Sync_Callback");
  // 2018-11-28 add below code for Warning "Failed to find match for field 'intensity'."
  // https://answers.ros.org/question/173396/losing-intensity-data-when-converting-between-sensor_msgspointcloud2-and-pclpointcloudt/
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  cloudMsg->fields[3].name = "intensity";
  syncMessages_ = SyncImageCloudPair(image_msg, cloudMsg);
  flag = true;
}

MessagesSync::~MessagesSync() {
  // TODO Auto-generated destructor stub
}

////////////////////////////////MyMessagesSync////////////////////////////////////////////////////////
MyMessagesSync::MyMessagesSync(ros::NodeHandle nh, std::string imagewithbboxes_topic_name, std::string lidar_topic_name):
    nodeHandle_(nh),
    subImageWithBBoxes_(nh, imagewithbboxes_topic_name, 20),
    subLidarData_(nh,lidar_topic_name, 10),
    flag(false),
    sync(MySyncPolicy(10), subImageWithBBoxes_, subLidarData_)
{
  //发布同步后的数据
  ROS_WARN("lidar and camera data synchronizing started");
  sync.registerCallback(boost::bind(&MyMessagesSync::cameraLidarCallback, this,_1, _2));
}

MyMessagesSync::SyncImageCloudPair MyMessagesSync::getSyncMessages()
{
  if(!flag)
    return SyncImageCloudPair();
  else {
    flag = false;
    return syncMessages_;
  }
}

void MyMessagesSync::cameraLidarCallback(const darknet_ros_msgs::ImageWithBBoxesConstPtr& image_msg,const sensor_msgs::PointCloud2ConstPtr& lidar_msg)
{
  ROS_WARN_THROTTLE(3, "Sync_Callback");
  // 2018-11-28 add below code for Warning "Failed to find match for field 'intensity'."
  // Reference: https://answers.ros.org/question/173396/losing-intensity-data-when-converting-between-sensor_msgspointcloud2-and-pclpointcloudt/
  sensor_msgs::PointCloud2Ptr cloudMsg(new sensor_msgs::PointCloud2(*lidar_msg));
  cloudMsg->fields[3].name = "intensity";
  syncMessages_ = SyncImageCloudPair(image_msg, cloudMsg);
  flag = true;
}

MyMessagesSync::~MyMessagesSync() {
  // TODO Auto-generated destructor stub
}

} /* namespace sensors_fusion */
