/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :　2019年6月9日
* Copyright    :
* Descriptoin  :Just for ros node test
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

#include <iv_slam_ros_msgs/TraversableArea.h>
#include "transform/rigid_transform.h"
//this project headers
#include "StructMovingTargetDefine.h"
#include "tracker/MovingTargetTrack.h"

#if 0
void  TraversableAreaOptimization(const iv_slam_ros_msgs::TraversableArea& received_traversable_area
                                                ,transform::Rigid3d global_vehicle_pose
                                                ,const std::string traversable_area_topic_name)//jkj
{
    double pose_x=0,pose_y=0;
    transform::geographic_to_grid(a, e2,
            (received_traversable_area.triD_submap_pose.position.y) * M_PI / 180,
            (received_traversable_area.triD_submap_pose.position.x) * M_PI / 180,
            &gps_zone, &hemi,
            &(pose_y),
            &(pose_x));

    transform::Rigid3d TwiDTraversableArea_pose = transform::Rigid3d(
            Eigen::Vector3d(pose_x-500000,
                    pose_y,
                    received_traversable_area.triD_submap_pose.position.z),
            Eigen::Quaternion<double>(
                    received_traversable_area.triD_submap_pose.orientation.w,
                    received_traversable_area.triD_submap_pose.orientation.x,
                    received_traversable_area.triD_submap_pose.orientation.y,
                    received_traversable_area.triD_submap_pose.orientation.z));



    Eigen::Vector3d submap_eular_pose = transform::toRollPitchYaw(
            TwiDTraversableArea_pose.rotation());
    Eigen::Vector3d global_vehicle_eular_pose = transform::toRollPitchYaw(
            global_vehicle_pose.rotation());
    global_vehicle_pose = transform::Rigid3d(
            global_vehicle_pose.translation(),
            transform::RollPitchYaw(submap_eular_pose.x(),
                    submap_eular_pose.y(), global_vehicle_eular_pose.z()));
    cv::Mat src_image = cv::Mat::zeros(received_traversable_area.height,
            received_traversable_area.width, CV_8U);
    cv::Mat traversable_area_optimized_image = src_image.clone();
    cv::Mat traversable_area_optimized_image_refine = src_image.clone();
    cv::Mat traversable_area_optimized_image_water = src_image.clone();
    int data_index = 0;

    int x_index2, y_index2;

//     double start_time = ros::Time::now().toSec();
    for (int j = 0; j < traversable_area_optimized_image.rows; j++) {
        int index_j = traversable_area_optimized_image.rows - 1 - j;
        uchar* srcdata = src_image.ptr<uchar>(index_j);
        uchar* testdata = traversable_area_optimized_image.ptr<uchar>(index_j);
        uchar* refinedata = traversable_area_optimized_image_refine.ptr<uchar>(index_j);
        uchar* waterdata = traversable_area_optimized_image_water.ptr<uchar>(index_j);
        for (int i = 0; i < traversable_area_optimized_image.cols; i++) {
            const auto& val = received_traversable_area.cells.at(data_index);
            srcdata[i] = val;
            if(val == SendDataType::KObstacle)
            {
                testdata[i] = val;
            }
            else if(val == SendDataType::KRefineObstacle)
                refinedata[i] = val;
            else if(val == SendDataType::KWaterObstacle)
                waterdata[i] = val;
            else if(val == SendDataType::KNegativeObstacle)
                testdata[i] = val;
//              if ((received_traversable_area.cells.at(data_index) == 2)) {
//
//                  testdata[i] = 255;
//
//              } else if (received_traversable_area.cells.at(data_index) == 1) {
//
//                  testdata[i] = 50;
//              }
            data_index++;
        }
    }




    auto vehicle_pose = TwiDTraversableArea_pose.inverse()
            * global_vehicle_pose.translation();

    cv::Point vehicle_position_in_image(
                    received_traversable_area.triD_submap_pose_image_index_x
                            + vehicle_pose.x()
                                    / received_traversable_area.resolution,
                                    traversable_area_optimized_image.rows - 1
                            - (received_traversable_area.triD_submap_pose_image_index_y
                                    + vehicle_pose.y()
                                            / received_traversable_area.resolution));


    traversable_area_optimization(traversable_area_optimized_image_refine,
            received_traversable_area.resolution, traversable_area_option_.vehicle_width,SendDataType::KRefineObstacle);
    traversable_area_optimization(traversable_area_optimized_image,
            received_traversable_area.resolution, traversable_area_option_.vehicle_width,SendDataType::KObstacle);
    traversable_area_optimization(traversable_area_optimized_image_water,
            received_traversable_area.resolution, traversable_area_option_.vehicle_width,SendDataType::KWaterObstacle);
//  known_area_extraction(traversable_area_optimized_image,vehicle_position_in_image
//          ,traversable_area_option_.known_radius/received_traversable_area.resolution);


/*******************************************publish*************************************************/
    iv_slam_ros_msgs::TraversableArea traversable_area_optimized=received_traversable_area;
        traversable_area_optimized.valid = false;
    data_index = 0;
    int count_for_valid = 0;
    for (int j = 0; j < traversable_area_optimized_image.rows; j++) {
        int index_j = traversable_area_optimized_image.rows - 1 - j;
        uchar* testdata = traversable_area_optimized_image.ptr<uchar>(index_j);
        uchar* refinedata = traversable_area_optimized_image_refine.ptr<uchar>(index_j);
        uchar* waterdata = traversable_area_optimized_image_water.ptr<uchar>(index_j);
        for (int i = 0; i < traversable_area_optimized_image.cols; i++) {
            if(testdata[i]==SendDataType::KObstacle)
                traversable_area_optimized.cells.at(data_index) = SendDataType::KObstacle;
            else if(testdata[i] == SendDataType::KNegativeObstacle)
                traversable_area_optimized.cells.at(data_index) = SendDataType::KNegativeObstacle;
            else if(refinedata[i]==SendDataType::KRefineObstacle)
                traversable_area_optimized.cells.at(data_index) = SendDataType::KRefineObstacle;
            else if(waterdata[i]==SendDataType::KWaterObstacle)
                traversable_area_optimized.cells.at(data_index) = SendDataType::KWaterObstacle;
            data_index++;
        }
    }
    bool detect_vehicle_in_map = false;
    if(traversable_area_topic_name == KOptimizedFinalTraversableAreaTopicName)
    {
        for(int i = 0; i < traversable_area_optimized.cells.size();i ++)
        {
            if(traversable_area_optimized.cells.at(i) != SendDataType::KObstacle && traversable_area_optimized.cells.at(i) != SendDataType::KRefineObstacle
               && traversable_area_optimized.cells.at(i) != SendDataType::KWaterObstacle && traversable_area_optimized.cells.at(i) != SendDataType::KNegativeObstacle)
            {
                count_for_valid++;
            }
        }
        int vehicle_in_map_x = received_traversable_area.triD_submap_pose_image_index_x + vehicle_pose.x() / received_traversable_area.resolution;
        int vehicle_in_map_y = traversable_area_optimized_image.rows - 1 - (received_traversable_area.triD_submap_pose_image_index_y + vehicle_pose.y() / received_traversable_area.resolution);
        if(vehicle_in_map_x > 0 && vehicle_in_map_x < traversable_area_optimized_image.rows && vehicle_in_map_y > 0 && vehicle_in_map_y < traversable_area_optimized_image.cols)
            detect_vehicle_in_map = true;
    }


    //0605zbc
        if(count_for_valid == traversable_area_optimized.cells.size() && detect_vehicle_in_map)
            traversable_area_optimized.valid = true;//空图且车在图里
        else if(count_for_valid == traversable_area_optimized.cells.size() && !detect_vehicle_in_map)
            traversable_area_optimized.valid = false;//空图且车不在图里
        else if(!(count_for_valid == traversable_area_optimized.cells.size()) && !detect_vehicle_in_map)
            traversable_area_optimized.valid = false;//图不空且车不在图里
        else
            traversable_area_optimized.valid = true;//图不空且车在图里
    ros::Time time_final = ros::Time::now();
    publishers[traversable_area_topic_name].publish(traversable_area_optimized);


    if(traversable_area_topic_name == KOptimizedFinalTraversableAreaTopicName)
    {
        special_area::special_area ditch_msg;
        ditch_msg.detected = false;
        ditch_msg.vertical = false;
        ditch_msg.center = {0,0};
        ditch_msg.angle = 0;
        ditch_msg.dis = 0;
        bool negative_flag = false;
        std::vector<int> left_down_width(0), left_down_height(0), right_down_width(0), right_down_height(0);
        for(int j = 1;j < traversable_area_optimized.height;j ++)
        {
            for(int i = 1; i < traversable_area_optimized.width;i ++)
            {
                int index = j * traversable_area_optimized.width + i;
                if(traversable_area_optimized.cells.at(index) == SendDataType::KNegativeObstacle)
                {
                    negative_flag = true;
                    int index_down = (j - 1) * traversable_area_optimized.width + i ;
                    int index_up = (j + 1) * traversable_area_optimized.width + i;
                    int index_left = j * traversable_area_optimized.width + i - 1;
                    int index_right = j * traversable_area_optimized.width + i + 1;
                    if(traversable_area_optimized.cells.at(index_down) != SendDataType::KNegativeObstacle && traversable_area_optimized.cells.at(index_up) == SendDataType::KNegativeObstacle
                       && traversable_area_optimized.cells.at(index_left) != SendDataType::KNegativeObstacle && traversable_area_optimized.cells.at(index_right) == SendDataType::KNegativeObstacle)
                    {
                        left_down_height.push_back(j);
                        left_down_width.push_back(i);
                    }
                    if(traversable_area_optimized.cells.at(index_down) != SendDataType::KNegativeObstacle && traversable_area_optimized.cells.at(index_up) == SendDataType::KNegativeObstacle
                       && traversable_area_optimized.cells.at(index_right) != SendDataType::KNegativeObstacle && traversable_area_optimized.cells.at(index_left) == SendDataType::KNegativeObstacle)
                    {
                        right_down_height.push_back(j);
                        right_down_width.push_back(i);
                    }

                }

            }
        }
        if(negative_flag)
        {
            int left_down_width_min(1000000);
            int left_down_height_min(0);
            int vehicle_pose_width = traversable_area_optimized.triD_submap_pose_image_index_x + vehicle_pose.x() / traversable_area_optimized.resolution;
            int vehicle_pose_height = traversable_area_optimized.triD_submap_pose_image_index_y + vehicle_pose.y() / traversable_area_optimized.resolution;
            for(int i = 0; i < left_down_height.size();i++)
            {
                if(left_down_height.at(i) < vehicle_pose_height + 20)//车前4米
                    continue;
                if(left_down_width_min > left_down_width.at(i))
                {
                    left_down_height_min = left_down_height.at(i);
                    left_down_width_min = left_down_width.at(i);
                }

            }
            int right_down_width_max(0);
            int right_down_height_max(0);
            for(int i = 0; i < right_down_height.size();i++)
            {
                if(right_down_height.at(i) < vehicle_pose_height + 20)
                    continue;
                if(right_down_width_max < right_down_width.at(i))
                {
                    right_down_height_max = right_down_height.at(i);
                    right_down_width_max = right_down_width.at(i);
                }
            }
            int negative_middle_width = (left_down_width_min + right_down_width_max) / 2;
            int negative_middle_height = (left_down_height_min + right_down_height_max) / 2;
            //统计出来的壕沟到车的距离
            double negative_distance = sqrt(pow((negative_middle_width - vehicle_pose_width) * traversable_area_optimized.resolution, 2) + pow((negative_middle_height - vehicle_pose_height)* traversable_area_optimized.resolution,2));
            double neagtive_x = (negative_middle_width - vehicle_pose_width) * traversable_area_optimized.resolution;//壕沟中心点到车的x
            double negative_y = (negative_middle_height - vehicle_pose_height) * traversable_area_optimized.resolution;//壕沟中心点到车的y
            double angle = atan(((right_down_height_max - left_down_height_min) * traversable_area_optimized.resolution) / ((right_down_width_max - left_down_width_min) * traversable_area_optimized.resolution));
            bool verticle = true;
            if(fabs(angle * 180 / M_PI) > angle_threshold)
                verticle = false;
            if(negative_y > 0)
            {

                ditch_msg.header = traversable_area_optimized.header;
                ditch_msg.dis = negative_distance;
                ditch_msg.angle = angle;
                ditch_msg.detected = negative_flag;
                ditch_msg.center = {float(neagtive_x), float(negative_y)};
                ditch_msg.vertical = verticle;

            }

        }
        publishers[KDitchTopicName].publish(ditch_msg);
    }

    LOG(INFO)<<traversable_area_topic_name<<" cost time-total:"<<(time_final-traversable_area_optimized.header.stamp).toSec();
    string filename = expand_user("~/traversable_area_optimization_time.txt");
    static std::ofstream file(filename);
    file<<std::fixed<<std::setprecision(4)<<traversable_area_optimized.header.stamp.toSec()
            <<" "<<(time_final - traversable_area_optimized.header.stamp).toSec()<<std::endl;
/*******************************************display*************************************************/
    cv::Mat Display_Image = cv::Mat::zeros(
            cv::Size(traversable_area_optimized_image.cols, traversable_area_optimized_image.rows), CV_8UC3);

    //  cv::imshow("out",out);

    for (int j = 0; j < Display_Image.rows; j++) {
        unsigned char* pdata = traversable_area_optimized_image.ptr<unsigned char>(j);
        unsigned char* refinedata = traversable_area_optimized_image_refine.ptr<unsigned char>(j);
        unsigned char* waterdata = traversable_area_optimized_image_water.ptr<unsigned char>(j);
        unsigned char* srcdata = src_image.ptr<unsigned char>(j);

        unsigned char* display_data = (unsigned char*) Display_Image.ptr<
                uchar>(j);
        for (int i = 0; i < Display_Image.cols; i++) {
            uchar pixel;
//          if (srcdata[i] == SendDataType::KUnknown)
//          {
//
//              display_data[3 * i] = 0;
//              display_data[3 * i + 1] = 0;
//              display_data[3 * i + 2] = 0;
//          }

            if (pdata[i] == SendDataType::KObstacle)
            {

                display_data[3 * i] = 255;
                display_data[3 * i + 1] = 255;
                display_data[3 * i + 2] = 255;
            }
            else if(pdata[i] == SendDataType::KNegativeObstacle)
            {
                display_data[3 * i] = 0;
                display_data[3 * i + 1] = 255;
                display_data[3 * i + 2] = 255;
            }
            else if (refinedata[i] == SendDataType::KRefineObstacle)
            {

                display_data[3 * i] = 60;
                display_data[3 * i + 1] = 60;
                display_data[3 * i + 2] = 255;
            }
            else if (waterdata[i] == SendDataType::KWaterObstacle)
            {

                display_data[3 * i] = 255;
                display_data[3 * i + 1] = 60;
                display_data[3 * i + 2] = 60;
            }
            else if(srcdata[i] == SendDataType::KNearUnknown)
            {

                display_data[3 * i] = 200;
                display_data[3 * i + 1] = 200;
                display_data[3 * i + 2] = 200;
            }
            else if(srcdata[i] == SendDataType::KPassibility)
            {

                display_data[3 * i] = 100;
                display_data[3 * i + 1] = 100;
                display_data[3 * i + 2] = 100;
            }
//          else
//          {
//              display_data[3 * i] = 0;
//              display_data[3 * i + 1] = 255;
//              display_data[3 * i + 2] = 0;
//          }
            //if(*pdata == 254)

        }
    }

    int linestep = 10 / received_traversable_area.resolution;
    int heightnum = Display_Image.rows / (linestep);
    int widthnum = Display_Image.cols / (linestep);
    for (int i = 0; i < heightnum; i++) {
        cv::line(Display_Image, cv::Point(0, linestep * i),
                cv::Point(Display_Image.cols - 1, linestep * i),
                cv::Scalar(255, 0, 0));
    }

    for (int i = 1; i < widthnum; i++) {
        cv::line(Display_Image, cv::Point(linestep * i, 0),
                cv::Point(linestep * i, Display_Image.rows - 1),
                cv::Scalar(255, 0, 0));
    }
    cv::circle(Display_Image,vehicle_position_in_image,
            5, cv::Scalar(0, 255, 255), -1);
    if(traversable_area_option_.display_on)
    {
        if(Display_Image.data != nullptr)
            cv::imshow(traversable_area_topic_name, Display_Image);
    }
}
#endif


class PostProcess
{
public:
  PostProcess(ros::NodeHandle& nodehandle):nodehandle_(nodehandle),
  processthread_(NULL),
  processthreadfinished_ (false)
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
    subTraversableArea_ = nodehandle_.subscribe<iv_slam_ros_msgs::TraversableArea>
    ("single_traversable_area_optimized_topic", 1, boost::bind(&PostProcess::traversableAreaCallBack,this,_1));
    // Begin main process
//    processthread_ = new boost::thread(boost::bind(&PostProcess::process,this));
  }

  void traversableAreaCallBack(const iv_slam_ros_msgs::TraversableAreaConstPtr& traversable_msg)
  {
    ROS_INFO("Enter in traversableAreaCallBack...");
    cout<<traversable_msg->cells.size()<<endl;
  }

  void process()
  {
    long long frame_counter = 0;
    while (!processthreadfinished_&&ros::ok()) {

    }//end while(!processthreadfinished_)
  }

protected:
  ros::Subscriber subTraversableArea_ ;//sub traversable area topic

  ros::NodeHandle nodehandle_;
  boost::thread* processthread_;
  bool processthreadfinished_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "dynamicobject");
  ros::NodeHandle nh;

  PostProcess postprocess(nh);
  ros::spin();

  return 0;
}

