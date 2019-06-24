
#include <iostream>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/locale.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/tokenizer.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <std_msgs/Bool.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <time.h>
#include <Eigen/Dense>

#include "kitti_track_label.h"
#include <darknet_ros_msgs/ImageWithBBoxes.h>

using namespace std;
using namespace pcl;
using namespace ros;
using namespace tf;

namespace po = boost::program_options;

static Eigen::Matrix4f RT_velo_to_cam;
static Eigen::Matrix4f transform_needed;
static Eigen::Matrix4f R_rect_00;
static Eigen::MatrixXf project_matrix(3,4);
Eigen::MatrixXf transform_matrix(3,4);

template <class PointT>
bool projectPoint2Image(const PointT& pointIn, const Eigen::MatrixXf& projectmatrix, cv::Point2f& point_project)
{
  //check project matrix size
  if(!(projectmatrix.rows()==3&&projectmatrix.cols()==4)) {
    std::cout<<"[WARN] project matrix size need be 3x4!"<<std::endl;
    return false;
  }

  //apply the projection operation
  Eigen::Vector4f point_3d(pointIn.x, pointIn.y, pointIn.z, 1.0);//define homogenious coordniate
  Eigen::Vector3f point_temp = projectmatrix*point_3d;
  //get image coordinates
  float x = static_cast<float>(point_temp[0]/point_temp[2]);
  float y = static_cast<float>(point_temp[1]/point_temp[2]);
  point_project = cv::Point2f(x,y);
  return true;
}

cv::Mat ProjectCloud2Image(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloudIn, const cv::Mat& imageIn, const Eigen::MatrixXf& projectmatrix)
{
  cv::Mat hsv_image, res_image;
  cv::cvtColor(imageIn, hsv_image, CV_BGR2HSV);
  int scale = 120, min_dis = 1, max_dis = 70;
  auto toColor = [&](const pcl::PointXYZI& point)->int{
    //1)calculate point distance
    float distance = std::sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
    //2)calculate color value, normalize values to (0 - scale) & close distance value has low value.
    int res = ((distance - min_dis) / (max_dis - min_dis))*scale;
    return res;
  };

  // plot color points using distance encode HSV color
  for(int i = 0;i<cloudIn->size();++i){
    if(cloudIn->points[i].y < 0)
      continue;
    int color = toColor(cloudIn->points[i]);
    cv::Point2f image_point;
    projectPoint2Image(cloudIn->points[i],projectmatrix, image_point);
    cv::circle(hsv_image, image_point, 2, cv::Scalar(color, 255, 255), -1);
  }
  cv::cvtColor(hsv_image, res_image,CV_HSV2BGR);
  return res_image;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TransformKittiCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr kitti_cloud, bool do_z_shift = false, float z_shift_value = 1.73)
{
  //do transformation
  Eigen::Affine3f transform_matrix = Eigen::Affine3f::Identity();
  // Define translation
  if(do_z_shift)
    transform_matrix.translation() << 0.0, 0.0, 1.73;
  else
    transform_matrix.translation() << 0.0, 0.0, 0.0;
  // The same rotation matrix as before; theta radians around Z axis
  transform_matrix.rotate (Eigen::AngleAxisf (M_PI/2, Eigen::Vector3f::UnitZ()));

  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*kitti_cloud, *transformed_cloud, transform_matrix);
  return transformed_cloud;
}

struct kitti_player_options
{
    string  path;
    float   frequency;        // publisher frequency. 1 > Kitti default 10Hz
    bool    all_data;         // publish everything
    bool    velodyne;         // publish velodyne point clouds /as PCL
    bool    gps;              // publish GPS sensor_msgs/NavSatFix    message
    bool    imu;              // publish IMU sensor_msgs/Imu Message  message
    bool    grayscale;        // publish
    bool    color;            // publish
    bool    viewer;           // enable CV viewer
    bool    timestamps;       // use KITTI timestamps;
    bool    sendTransform;    // publish velodyne TF IMU 3DOF orientation wrt fixed frame
    bool    stereoDisp;       // use precalculated stereoDisparities
    bool    viewDisparities;  // view use precalculated stereoDisparities
    bool    synchMode;        // start with synchMode on (wait for message to send next frame)
    unsigned int startFrame;  // start the replay at frame ...
    string gpsReferenceFrame; // publish GPS points into RVIZ as RVIZ Markers
};


bool waitSynch = false; /// Synch mode variable, refs #600

/**
 * @brief synchCallback
 * @param msg (boolean)
 *
 * if a TRUE message is received, TRUE is interpreted as "publish a new frame".
 * Then waitSynh variable is set to FALSE, and an iteration of the KittiPlayer
 * main loop is executed.
 */
void synchCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_INFO_STREAM("Synch received");
    if (msg->data)
        waitSynch = false;
}

/**
 * @brief publish_velodyne
 * @param pub The ROS publisher as reference
 * @param infile file with data to publish
 * @param header Header to use to publish the message
 * @return 1 if file is correctly readed, 0 otherwise
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr publish_velodyne(ros::Publisher &pub, string infile, std_msgs::Header *header)
{
    fstream input(infile.c_str(), ios::in | ios::binary);
    if (!input.good())
    {
        ROS_ERROR_STREAM ( "Could not read file: " << infile );
        return 0;
    }
    else
    {
        ROS_DEBUG_STREAM ("reading " << infile);
        input.seekg(0, ios::beg);

        pcl::PointCloud<pcl::PointXYZI>::Ptr points (new pcl::PointCloud<pcl::PointXYZI>);

        int i;
        for (i = 0; input.good() && !input.eof(); i++)
        {
            pcl::PointXYZI point;
            input.read((char *) &point.x, 3 * sizeof(float));
            input.read((char *) &point.intensity, sizeof(float));
            points->push_back(point);
        }
        input.close();

        //workaround for the PCL headers... http://wiki.ros.org/hydro/Migration#PCL
        sensor_msgs::PointCloud2 pc2;

        pc2.header.frame_id = "velo_link"; //ros::this_node::getName();
        pc2.header.stamp = header->stamp;
        points->header = pcl_conversions::toPCL(pc2.header);
        pub.publish(points);

        return points;
    }
}

/**
 * @brief getCalibration
 * @param dir_root
 * @param camera_name
 * @param K double K[9]  - Calibration Matrix
 * @param D double D[5]  - Distortion Coefficients
 * @param R double R[9]  - Rectification Matrix
 * @param P double P[12] - Projection Matrix Rectified (u,v,w) = P * R * (x,y,z,q)
 * @return 1: file found, 0: file not found
 *
 *  from: http://kitti.is.tue.mpg.de/kitti/devkit_raw_data.zip
 *  calib_cam_to_cam.txt: Camera-to-camera calibration
 *
 *    - S_xx: 1x2 size of image xx before rectification
 *    - K_xx: 3x3 calibration matrix of camera xx before rectification
 *    - D_xx: 1x5 distortion vector of camera xx before rectification
 *    - R_xx: 3x3 rotation matrix of camera xx (extrinsic)
 *    - T_xx: 3x1 translation vector of camera xx (extrinsic)
 *    - S_rect_xx: 1x2 size of image xx after rectification
 *    - R_rect_xx: 3x3 rectifying rotation to make image planes co-planar
 *    - P_rect_xx: 3x4 projection matrix after rectification
 */
int getCalibration(string dir_root, string camera_name, double* K, std::vector<double> & D, double *R, double* P)
{
    string calib_cam_to_cam = dir_root + "calib_cam_to_cam.txt";
    ifstream file_c2c(calib_cam_to_cam.c_str());
    if (!file_c2c.is_open())
        return false;

    ROS_INFO_STREAM("Reading camera" << camera_name << " calibration from " << calib_cam_to_cam);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    string line = "";
    unsigned char index = 0;
    tokenizer::iterator token_iterator;

    while (getline(file_c2c, line))
    {
        // Parse string phase 1, tokenize it using Boost.
        tokenizer tok(line, sep);

        // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.
        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("K_") + camera_name + string(":"))).c_str()) == 0) //Calibration Matrix
        {
            index = 0; //should be 9 at the end
            ROS_DEBUG_STREAM("K_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                K[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

        // EXPERIMENTAL: use with unrectified images

        //        token_iterator=tok.begin();
        //        if (strcmp((*token_iterator).c_str(),((string)(string("D_")+camera_name+string(":"))).c_str())==0) //Distortion Coefficients
        //        {
        //            index=0; //should be 5 at the end
        //            ROS_DEBUG_STREAM("D_" << camera_name);
        //            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
        //            {
        ////                std::cout << *token_iterator << '\n';
        //                D[index++]=boost::lexical_cast<double>(*token_iterator);
        //            }
        //        }

        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("R_") + camera_name + string(":"))).c_str()) == 0) //Rectification Matrix
        {
            index = 0; //should be 12 at the end
            ROS_DEBUG_STREAM("R_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                R[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

        token_iterator = tok.begin();
        if (strcmp((*token_iterator).c_str(), ((string)(string("P_rect_") + camera_name + string(":"))).c_str()) == 0) //Projection Matrix Rectified
        {
            index = 0; //should be 12 at the end
            ROS_DEBUG_STREAM("P_rect_" << camera_name);
            for (token_iterator++; token_iterator != tok.end(); token_iterator++)
            {
                //std::cout << *token_iterator << '\n';
                P[index++] = boost::lexical_cast<double>(*token_iterator);
            }
        }

    }
    ROS_INFO_STREAM("... ok");
    return true;
}

int getCalibration2(string full_filename_calibration)
{
  ifstream file_c2c(full_filename_calibration.c_str());
  if (!file_c2c.is_open())
    return false;

  ROS_INFO_STREAM("Reading calibration from " << full_filename_calibration);

  typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
  boost::char_separator<char> sep {" "};

  string line = "";
  unsigned char index = 0;
  tokenizer::iterator token_iterator;

  double P2[12], R_rect[12], Tr_velo_cam[12];
  while (getline(file_c2c, line))
  {
    // Parse string phase 1, tokenize it using Boost.
    tokenizer tok(line, sep);

    // Move the iterator at the beginning of the tokenize vector and check for K/D/R/P matrices.
    token_iterator = tok.begin();
    if (strcmp((*token_iterator).c_str(), ((string)(string("P2:"))).c_str()) == 0) //Calibration Matrix
    {
      index = 0; //should be 12 at the end
      for (token_iterator++; token_iterator != tok.end(); token_iterator++)
      {
        //std::cout << *token_iterator << '\n';
        P2[index++] = boost::lexical_cast<double>(*token_iterator);
      }
    }

    token_iterator = tok.begin();
    if (strcmp((*token_iterator).c_str(), ((string)(string("R_rect"))).c_str()) == 0) //Rectification Matrix
    {
      index = 0; //should be 12 at the end
      for (token_iterator++; token_iterator != tok.end(); token_iterator++)
      {
        //std::cout << *token_iterator << '\n';
        R_rect[index++] = boost::lexical_cast<double>(*token_iterator);
      }
    }

    token_iterator = tok.begin();
    if (strcmp((*token_iterator).c_str(), ((string)(string("Tr_velo_cam"))).c_str()) == 0) //Projection Matrix Rectified
    {
      index = 0; //should be 12 at the end
      for (token_iterator++; token_iterator != tok.end(); token_iterator++)
      {
        //std::cout << *token_iterator << '\n';
        Tr_velo_cam[index++] = boost::lexical_cast<double>(*token_iterator);
      }
    }
  }

  project_matrix << P2[0], P2[1], P2[2], P2[3],
                    P2[4], P2[5], P2[6], P2[7],
                    P2[8], P2[9], P2[10], P2[11];
  R_rect_00 << R_rect[0], R_rect[1], R_rect[2], 0.0,
               R_rect[3],R_rect[4], R_rect[5], 0.0,
               R_rect[6], R_rect[7],R_rect[8], 0.0,
               0.0,  0.0, 0.0, 1.0;
  RT_velo_to_cam << Tr_velo_cam[0], Tr_velo_cam[1], Tr_velo_cam[2], Tr_velo_cam[3],
                    Tr_velo_cam[4], Tr_velo_cam[5], Tr_velo_cam[6], Tr_velo_cam[7],
                    Tr_velo_cam[8], Tr_velo_cam[9], Tr_velo_cam[10], Tr_velo_cam[11],
                    0.0,  0.0, 0.0, 1.0;
  cout<<"============================"<<endl;
  cout<<project_matrix<<endl;
  cout<<R_rect_00<<endl;
  cout<<RT_velo_to_cam<<endl;
  ROS_INFO_STREAM("... ok");
  return true;

}

void drawBBoxes(cv::Mat& img, const std::vector<ObjectDetect> &outRcs)
{
  // draw detection results
  for(const auto& rect:outRcs) {
    if(rect.type == "Car")
      cv::rectangle(img, rect.bbox, cv::Scalar(142, 0, 0), 2);
    else if(rect.type == "Pedestrian")
      cv::rectangle(img, rect.bbox, cv::Scalar(60, 20 ,220), 2);
    else if(rect.type == "Cyclist")
      cv::rectangle(img, rect.bbox, cv::Scalar(32, 11, 119), 2);
    else
      cv::rectangle(img, rect.bbox, cv::Scalar(255, 255, 255), 2);
    // Draw occluded text
    cv::putText(img, to_string(rect.occluded),rect.bbox.tl(), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0,255,255));
    cv::putText(img, rect.type, cv::Point(rect.bbox.tl().x + 8, rect.bbox.tl().y - 2), cv::FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(0,255,255));
  }
  cv::imshow("bboxes", img);
  cv::waitKey(5);
}

bool publishImageWithBBoxes(ros::Publisher &pub,
                            const cv::Mat& raw_image,
                            const std::vector<ObjectDetect>& bboxes,
                            std_msgs::Header *header)
{
  if (pub.getNumSubscribers() < 1)
    return false;

  darknet_ros_msgs::BoundingBoxes boundingBoxesResults_;
  //To ROS msg
  for (int i = 0; i < bboxes.size(); i++) {//loop every object class
      darknet_ros_msgs::BoundingBox boundingBox;

      int xmin = bboxes[i].bbox.tl().x;
      int ymin = bboxes[i].bbox.tl().y;
      int xmax = bboxes[i].bbox.br().x;
      int ymax = bboxes[i].bbox.br().y;

      boundingBox.Class = bboxes[i].type;
      boundingBox.probability = 1.0;
      boundingBox.xmin = xmin;
      boundingBox.ymin = ymin;
      boundingBox.xmax = xmax;
      boundingBox.ymax = ymax;
      boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
  }

  boundingBoxesResults_.header.stamp = header->stamp;
  boundingBoxesResults_.header.frame_id = "detection";

  darknet_ros_msgs::ImageWithBBoxes image_with_bboxes;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = header->stamp;
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = raw_image;
  //to darknet_ros_msgs::ImageWithBBoxes message type
  image_with_bboxes.header = *header;
  image_with_bboxes.image = *cvImage.toImageMsg();
  image_with_bboxes.bboxes = boundingBoxesResults_;
  pub.publish(image_with_bboxes);
  ROS_DEBUG("Raw image with bounding boxes infomation have been published.");
  return true;
}

int getGPS(const std::vector<string>& lines, int entries, sensor_msgs::NavSatFix *ros_msgGpsFix, std_msgs::Header *header)
{
  // Get line
    std::string line = lines.at(entries);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgGpsFix->header.frame_id = ros::this_node::getName();
    ros_msgGpsFix->header.stamp = header->stamp;

    ros_msgGpsFix->latitude  = boost::lexical_cast<double>(s[0]);
    ros_msgGpsFix->longitude = boost::lexical_cast<double>(s[1]);
    ros_msgGpsFix->altitude  = boost::lexical_cast<double>(s[2]);

    ros_msgGpsFix->position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;
    for (int i = 0; i < 9; i++)
        ros_msgGpsFix->position_covariance[i] = 0.0f;

    ros_msgGpsFix->position_covariance[0] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[4] = boost::lexical_cast<double>(s[23]);
    ros_msgGpsFix->position_covariance[8] = boost::lexical_cast<double>(s[23]);

    ros_msgGpsFix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    ros_msgGpsFix->status.status  = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;

    return 1;
}

int getIMU(const std::vector<string>& lines, int entries, sensor_msgs::Imu *ros_msgImu, std_msgs::Header *header)
{
    std::string line = lines.at(entries);

    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
    boost::char_separator<char> sep {" "};

    tokenizer tok(line, sep);
    vector<string> s(tok.begin(), tok.end());

    ros_msgImu->header.frame_id = ros::this_node::getName();
    ros_msgImu->header.stamp = header->stamp;

    //    - ax:      acceleration in x, i.e. in direction of vehicle front (m/s^2)
    //    - ay:      acceleration in y, i.e. in direction of vehicle left (m/s^2)
    //    - az:      acceleration in z, i.e. in direction of vehicle top (m/s^2)
    ros_msgImu->linear_acceleration.x = boost::lexical_cast<double>(s[11]);
    ros_msgImu->linear_acceleration.y = boost::lexical_cast<double>(s[12]);
    ros_msgImu->linear_acceleration.z = boost::lexical_cast<double>(s[13]);

    //    - vf:      forward velocity, i.e. parallel to earth-surface (m/s)
    //    - vl:      leftward velocity, i.e. parallel to earth-surface (m/s)
    //    - vu:      upward velocity, i.e. perpendicular to earth-surface (m/s)
    ros_msgImu->angular_velocity.x = boost::lexical_cast<double>(s[8]);
    ros_msgImu->angular_velocity.y = boost::lexical_cast<double>(s[9]);
    ros_msgImu->angular_velocity.z = boost::lexical_cast<double>(s[10]);

    //    - roll:    roll angle (rad),  0 = level, positive = left side up (-pi..pi)
    //    - pitch:   pitch angle (rad), 0 = level, positive = front down (-pi/2..pi/2)
    //    - yaw:     heading (rad),     0 = east,  positive = counter clockwise (-pi..pi)
    tf::Quaternion q = tf::createQuaternionFromRPY(   boost::lexical_cast<double>(s[3]),
                                                      boost::lexical_cast<double>(s[4]),
                                                      boost::lexical_cast<double>(s[5])
                                                  );
    ros_msgImu->orientation.x = q.getX();
    ros_msgImu->orientation.y = q.getY();
    ros_msgImu->orientation.z = q.getZ();
    ros_msgImu->orientation.w = q.getW();

    return 1;
}

/// Cartesian coordinates struct, refs# 522
struct Xy
{
    double x;
    double y;
};

/** Conversion between geographic and UTM coordinates
    Adapted from:  http://www.uwgb.edu/dutchs/UsefulData/ConvertUTMNoOZ.HTM
    Refs# 522
**/
Xy latlon2xy_helper(double lat, double lngd)
{
    // WGS 84 datum
    double eqRad = 6378137.0;
    double flat = 298.2572236;

    // constants used in calculations:
    double a = eqRad;           // equatorial radius in meters
    double f = 1.0 / flat;        // polar flattening
    double b = a * (1.0 - f);     // polar radius
    double e = sqrt(1.0 - (pow(b, 2) / pow(a, 2))); // eccentricity
    double k0 = 0.9996;
    double drad = M_PI / 180.0;

    double phi = lat * drad;   // convert latitude to radians
    double utmz = 1.0 + floor((lngd + 180.0) / 6.0); // longitude to utm zone
    double zcm = 3.0 + 6.0 * (utmz - 1.0) - 180.0;     // central meridian of a zone
    double esq = (1.0 - (b / a) * (b / a));
    double e0sq = e * e / (1.0 - e * e);
    double M = 0.0;
    double M0 = 0.0;
    double N = a / sqrt(1.0 - pow(e * sin(phi), 2));
    double T = pow(tan(phi), 2);
    double C = e0sq * pow(cos(phi), 2);
    double A = (lngd - zcm) * drad * cos(phi);

    // calculate M (USGS style)
    M = phi * (1.0 - esq * (1.0 / 4.0 + esq * (3.0 / 64.0 + 5.0 * esq / 256.0)));
    M = M - sin(2.0 * phi) * (esq * (3.0 / 8.0 + esq * (3.0 / 32.0 + 45.0 * esq / 1024.0)));
    M = M + sin(4.0 * phi) * (esq * esq * (15.0 / 256.0 + esq * 45.0 / 1024.0));
    M = M - sin(6.0 * phi) * (esq * esq * esq * (35.0 / 3072.0));
    M = M * a; // Arc length along standard meridian

    // now we are ready to calculate the UTM values...
    // first the easting (relative to CM)
    double x = k0 * N * A * (1.0 + A * A * ((1.0 - T + C) / 6.0 + A * A * (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * e0sq) / 120.0));
    x = x + 500000.0; // standard easting

    // now the northing (from the equator)
    double y = k0 * (M - M0 + N * tan(phi) * (A * A * (1.0 / 2.0 + A * A * ((5.0 - T + 9.0 * C + 4.0 * C * C) / 24.0 + A * A * (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * e0sq) / 720.0))));
    if (y < 0)
    {
        y = 10000000.0 + y; // add in false northing if south of the equator
    }
    double easting  = x;
    double northing = y;

    Xy coords;
    coords.x = easting;
    coords.y = northing;

    return coords;
}



/**
 * @brief parseTime
 * @param timestamp in Epoch
 * @return std_msgs::Header with input timpestamp converted from file input
 *
 * Epoch time conversion
 * http://www.epochconverter.com/programming/functions-c.php
 */
std_msgs::Header parseTime(string timestamp)
{

    std_msgs::Header header;

    // example: 2011-09-26 13:21:35.134391552
    //          01234567891111111111222222222
    //                    0123456789012345678
    struct tm t = {0};  // Initalize to all 0's
    t.tm_year = boost::lexical_cast<int>(timestamp.substr(0, 4)) - 1900;
    t.tm_mon  = boost::lexical_cast<int>(timestamp.substr(5, 2)) - 1;
    t.tm_mday = boost::lexical_cast<int>(timestamp.substr(8, 2));
    t.tm_hour = boost::lexical_cast<int>(timestamp.substr(11, 2));
    t.tm_min  = boost::lexical_cast<int>(timestamp.substr(14, 2));
    t.tm_sec  = boost::lexical_cast<int>(timestamp.substr(17, 2));
    t.tm_isdst = -1;
    time_t timeSinceEpoch = mktime(&t);

    header.stamp.sec  = timeSinceEpoch;
    header.stamp.nsec = boost::lexical_cast<int>(timestamp.substr(20, 8));

    return header;
}


/**
 * @brief main Kitti_player, a player for KITTI raw datasets
 * @param argc
 * @param argv
 * @return 0 and ros::shutdown at the end of the dataset, -1 if errors
 *
 * Allowed options:
 *   -h [ --help ]                       help message
 *   -d [ --directory  ] arg             *required* - path to the kitti dataset Directory
 *   -f [ --frequency  ] arg (=1)        set replay Frequency
 *   -a [ --all        ] [=arg(=1)] (=0) replay All data
 *   -v [ --velodyne   ] [=arg(=1)] (=0) replay Velodyne data
 *   -g [ --gps        ] [=arg(=1)] (=0) replay Gps data
 *   -i [ --imu        ] [=arg(=1)] (=0) replay Imu data
 *   -G [ --grayscale  ] [=arg(=1)] (=0) replay Stereo Grayscale images
 *   -C [ --color      ] [=arg(=1)] (=0) replay Stereo Color images
 *   -V [ --viewer     ] [=arg(=1)] (=0) enable image viewer
 *   -T [ --timestamps ] [=arg(=1)] (=0) use KITTI timestamps
 *   -s [ --stereoDisp ] [=arg(=1)] (=0) use pre-calculated disparities
 *   -D [ --viewDisp   ] [=arg(=1)] (=0) view loaded disparity images
 *   -F [ --frame      ] [=arg(=0)] (=0) start playing at frame ...
 *
 * Datasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php
 */
int main(int argc, char **argv)
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
  project_matrix<<7.215377e+02,0.000000e+00,6.095593e+02,4.485728e+01,
                  0.000000e+00,7.215377e+02,1.728540e+02,2.163791e-01,
                  0.000000e+00,0.000000e+00,1.000000e+00,2.745884e-03;


    kitti_player_options options;
    string sequence_num; // string for easily concanate
    po::variables_map vm;

    po::options_description desc("Kitti_player, a player for KITTI raw datasets\nDatasets can be downloaded from: http://www.cvlibs.net/datasets/kitti/raw_data.php\n\nAllowed options", 200);
    desc.add_options()
    ("help,h"                                                                                                    ,  "help message")
    ("directory ,d",  po::value<string>       (&options.path)->required()                                        ,  "*required* - path to the kitti dataset Directory")
    ("sequence  ,s",  po::value<string>       (&sequence_num)->required()                                       ,  "*required* - want to handle which sequnce, e.g. 0000")
    ("frequency ,f",  po::value<float>        (&options.frequency)        ->default_value(1.0)                     ,  "set replay Frequency")
    ("all       ,a",  po::value<bool>         (&options.all_data)         ->default_value(0) ->implicit_value(1)   ,  "replay All data")
    ("velodyne  ,v",  po::value<bool>         (&options.velodyne)         ->default_value(0) ->implicit_value(1)   ,  "replay Velodyne data")
    ("gps       ,g",  po::value<bool>         (&options.gps)              ->default_value(0) ->implicit_value(1)   ,  "replay Gps data")
    ("imu       ,i",  po::value<bool>         (&options.imu)              ->default_value(0) ->implicit_value(1)   ,  "replay Imu data")
    ("color     ,C",  po::value<bool>         (&options.color)            ->default_value(0) ->implicit_value(1)   ,  "replay Stereo Color images")
    ("viewer    ,V",  po::value<bool>         (&options.viewer)           ->default_value(0) ->implicit_value(1)   ,  "enable image viewer")
    ("frame     ,F",  po::value<unsigned int> (&options.startFrame)       ->default_value(0) ->implicit_value(0)   ,  "start playing at frame...")
    ("gpsPoints ,p",  po::value<string>       (&options.gpsReferenceFrame)->default_value("")                      ,  "publish GPS/RTK markers to RVIZ, having reference frame as <reference_frame> [example: -p map]")
    ("synchMode ,S",  po::value<bool>         (&options.synchMode)        ->default_value(0) ->implicit_value(1)   ,  "Enable Synch mode (wait for signal to load next frame [std_msgs/Bool data: true]")
    ;

    try // parse options
    {
        po::parsed_options parsed = po::command_line_parser(argc, argv).options(desc).allow_unregistered().run();
        po::store(parsed, vm);
        po::notify(vm);

        vector<string> to_pass_further = po::collect_unrecognized(parsed.options, po::include_positional);
    }
    catch (...)
    {
        cerr << desc << endl;

        cout << "kitti_player needs a directory tree like the following:" << endl;
        cout << "└── training" << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── 0000              " << endl;
        cout << "    │   └── 0001              " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── 0000.txt          " << endl;
        cout << "    │   └── 0001.txt          " << endl;
        cout << "    ├── velodyne              " << endl;
        cout << "    │   └── 0000              " << endl;
        cout << "    │   └── 0001              " << endl;

        ROS_WARN_STREAM("Parse error, shutting down node\n");
        return -1;
    }

    ros::init(argc, argv, "kitti_player");
    ros::NodeHandle node("kitti");
    ros::Rate loop_rate(options.frequency);

    /// This sets the logger level; use this to disable all ROS prints
    if ( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) )
        ros::console::notifyLoggerLevelsChanged();
    else
        std::cout << "Error while setting the logger level!" << std::endl;

    DIR *dir;
    struct dirent *ent;
    unsigned int total_entries = 0;        //number of elements to be played
    unsigned int entries_played  = 0;      //number of elements played until now
    unsigned int len = 0;                  //counting elements support variable
    string dir_root             ;
    string dir_image02          ;
    string full_filename_image02;
    string dir_label02          ; // Image labels
    string dir_oxts             ;
    string dir_calib            ;
    string full_filename_calibration;
    string full_filename_oxts;
    string dir_timestamp_oxts;
    string dir_velodyne_points  ;
    string full_filename_velodyne;
    string dir_timestamp_velodyne; //average of start&end (time of scan)
    string str_support;
    cv::Mat cv_image02;
    cv::Mat cv_image03;
    cv::Mat cv_image04;
    std_msgs::Header header_support;
    KittiTrackLabel* kitti_track_label;

    image_transport::ImageTransport it(node);
    image_transport::CameraPublisher pub02 = it.advertiseCamera("camera_color_left/image_raw", 1);

    sensor_msgs::Image ros_msg02;
    sensor_msgs::Image ros_msg03;


//    sensor_msgs::CameraInfo ros_cameraInfoMsg;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera02;
    sensor_msgs::CameraInfo ros_cameraInfoMsg_camera03;

    cv_bridge::CvImage cv_bridge_img;

    ros::Publisher map_pub           = node.advertise<pcl::PointCloud<pcl::PointXYZ> >  ("velo/pointcloud", 1, true);
    ros::Publisher gps_pub           = node.advertise<sensor_msgs::NavSatFix>           ("oxts/gps", 1, true);
    ros::Publisher gps_pub_initial   = node.advertise<sensor_msgs::NavSatFix>           ("oxts/gps_initial", 1, true);
    ros::Publisher imu_pub           = node.advertise<sensor_msgs::Imu>                 ("oxts/imu", 1, true);
    ros::Publisher raw_image_with_bboxes_pub = node.advertise<darknet_ros_msgs::ImageWithBBoxes> ("/darknet_ros/image_with_bboxes", 1);

    sensor_msgs::NavSatFix  ros_msgGpsFix;
    sensor_msgs::NavSatFix  ros_msgGpsFixInitial;   // This message contains the first reading of the file
    bool                    firstGpsData = true;    // Flag to store the ros_msgGpsFixInitial message
    sensor_msgs::Imu        ros_msgImu;


    ros::Subscriber sub = node.subscribe("/kitti_player/synch", 1, synchCallback);    // refs #600

    if (vm.count("help"))
    {
        cout << desc << endl;

        cout << "kitti_player needs a directory tree like the following:" << endl;
        cout << "└── training" << endl;
        cout << "    ├── image_02              " << endl;
        cout << "    │   └── 0000              " << endl;
        cout << "    │   └── 0001              " << endl;
        cout << "    ├── oxts                  " << endl;
        cout << "    │   └── 0000.txt          " << endl;
        cout << "    │   └── 0001.txt          " << endl;
        cout << "    ├── velodyne              " << endl;
        cout << "    │   └── 0000              " << endl;
        cout << "    │   └── 0001              " << endl;

        return 1;
    }

    if (!(options.all_data || options.color || options.gps || options.grayscale || options.imu || options.velodyne))
    {
        ROS_WARN_STREAM("Job finished without playing the dataset. No 'publishing' parameters provided");
        node.shutdown();
        return 1;
    }

    ROS_WARN_STREAM("Your have choose play back "<<sequence_num << " data");

    dir_root             = options.path;
    dir_image02          = options.path;
    dir_oxts             = options.path;
    dir_velodyne_points  = options.path;

    (*(options.path.end() - 1) != '/' ? dir_root            = options.path + "/"                      : dir_root            = options.path);
    (*(options.path.end() - 1) != '/' ? dir_image02         = options.path + "/image_02/"        : dir_image02         = options.path + "image_02/");
    (*(options.path.end() - 1) != '/' ? dir_calib           = options.path + "/calib/"        :    dir_calib         = options.path + "calib/");
    (*(options.path.end() - 1) != '/' ? dir_label02         = options.path + "/label_02/"        : dir_label02         = options.path + "label_02/");
    (*(options.path.end() - 1) != '/' ? dir_oxts            = options.path + "/oxts/"            : dir_oxts            = options.path + "oxts/");
    (*(options.path.end() - 1) != '/' ? dir_velodyne_points = options.path + "/velodyne/" : dir_velodyne_points = options.path + "velodyne/");

    // Check all the directories
    if (
        (options.all_data       && (
                                       (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_oxts.c_str())               == NULL) ||
                                       (opendir(dir_label02.c_str())            == NULL) ||
                                       (opendir(dir_calib.c_str())              == NULL) ||
                                       (opendir(dir_velodyne_points.c_str())    == NULL)))
        ||
        (options.color          && (   (opendir(dir_image02.c_str())            == NULL) ||
                                       (opendir(dir_label02.c_str())            == NULL)))
        ||
        (options.imu            && (   (opendir(dir_oxts.c_str())               == NULL)))
        ||
        (options.gps            && (   (opendir(dir_oxts.c_str())               == NULL)))
        ||
        (options.velodyne       && (   (opendir(dir_velodyne_points.c_str())    == NULL)))
    )
    {
        ROS_ERROR("Incorrect tree directory , use --help for details");
        node.shutdown();
        return -1;
    }
    else
    {
        ROS_INFO_STREAM ("Checking directories...");
        ROS_INFO_STREAM (options.path << "\t[OK]");
    }

    //count elements in the folder
    string filename_image02 = dir_image02 + sequence_num + "/";
    if (options.all_data)//if replay all data in data folders
    {
        dir = opendir(filename_image02.c_str());
        while ((ent = readdir(dir)))
        {
            //skip . & ..
            len = strlen (ent->d_name);
            //skip . & ..
            if (len > 2)
                total_entries++;
        }
        closedir (dir);
    }
    else
    {
        bool done = false;
        if (!done && options.color)
        {
            total_entries = 0;
            dir = opendir(filename_image02.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.gps)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.imu)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
        if (!done && options.velodyne)
        {
            total_entries = 0;
            dir = opendir(dir_oxts.c_str());
            while ((ent = readdir(dir)))
            {
                //skip . & ..
                len = strlen (ent->d_name);
                //skip . & ..
                if (len > 2)
                    total_entries++;
            }
            closedir (dir);
            done = true;
        }
    }

    // Check options.startFrame and total_entries
    if (options.startFrame > total_entries)
    {
        ROS_ERROR("Error, start number > total entries in the dataset");
        node.shutdown();
        return -1;
    }
    else
    {
        entries_played = options.startFrame;
        ROS_INFO_STREAM("The entry point (frame number) is: " << entries_played);
        ROS_WARN_STREAM("The total frames numberis: " << total_entries);
    }

    if (options.viewer)
    {
        ROS_INFO_STREAM("Opening CV viewer(s)");
        if (options.color || options.all_data)
        {
            ROS_DEBUG_STREAM("color||all " << options.color << " " << options.all_data);
            cv::namedWindow("CameraSimulator Color Viewer", CV_WINDOW_AUTOSIZE);
            full_filename_image02 = dir_image02 + sequence_num + boost::str(boost::format("%06d") % 0 ) + ".png";
            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
            cv::waitKey(5);
        }
        ROS_INFO_STREAM("Opening CV viewer(s)... OK");
    }

    // CAMERA INFO SECTION: read one for all
    ros_cameraInfoMsg_camera02.header.stamp = ros::Time::now();
    ros_cameraInfoMsg_camera02.header.frame_id = ros::this_node::getName();
    ros_cameraInfoMsg_camera02.height = 0;
    ros_cameraInfoMsg_camera02.width  = 0;
    //ros_cameraInfoMsg_camera02.D.resize(5);
    //ros_cameraInfoMsg_camera02.distortion_model=sensor_msgs::distortion_models::PLUMB_BOB;

    //get color camera calibration matrix and set the image size

    if (options.color || options.all_data)
    {
      //Assume same height/width for the camera pair
      full_filename_image02 = dir_image02 + sequence_num + "/" + boost::str(boost::format("%06d") % 0 ) + ".png";
      cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);
      cv::waitKey(5);
      ros_cameraInfoMsg_camera03.height = ros_cameraInfoMsg_camera02.height = cv_image02.rows;// -1;TODO: CHECK, qui potrebbe essere -1
      ros_cameraInfoMsg_camera03.width  = ros_cameraInfoMsg_camera02.width  = cv_image02.cols;// -1;

      // Read detection label
      string full_fliename_label02 = dir_label02 + sequence_num + ".txt";
      kitti_track_label = new KittiTrackLabel(full_fliename_label02, cv::Size(cv_image02.size()));
    }

    // Load calibration matrix anyway
    full_filename_calibration = dir_calib + sequence_num + ".txt";
    if(!getCalibration2(full_filename_calibration)) {
      ROS_ERROR_STREAM("Error reading  calibration");
      node.shutdown();
      return -1;
    }



/******************************************************************************
 *  This is the main KITTI_PLAYER Loop
 */
    // display progress bar
    boost::progress_display progress(total_entries) ;
    do
    {
        // this refs #600 synchMode
        if (options.synchMode)
        {
            if (waitSynch == true)
            {
                //ROS_DEBUG_STREAM("Waiting for synch...");
                ros::spinOnce();
                continue;
            }
            else
            {
                ROS_DEBUG_STREAM("Run after received synch...");
                waitSynch = true;
            }
        }

        // single timestamp for all published stuff
        Time current_timestamp = ros::Time::now();

        //publish 02 color camera image
        if (options.color || options.all_data)
        {
            full_filename_image02 = dir_image02 + sequence_num + "/" + boost::str(boost::format("%06d") % entries_played ) + ".png";
            ROS_DEBUG_STREAM ( full_filename_image02 << endl << endl);

            cv_image02 = cv::imread(full_filename_image02, CV_LOAD_IMAGE_UNCHANGED);

            if ( (cv_image02.data == NULL) )
            {
                ROS_ERROR_STREAM("Error reading color images 02");
                ROS_ERROR_STREAM(full_filename_image02 << endl);
                node.shutdown();
                return -1;
            }

            if (options.viewer)
            {
                //display the left image only
                cv::imshow("CameraSimulator Color Viewer", cv_image02);
                //give some time to draw images
                cv::waitKey(5);
            }

            cv_bridge_img.encoding = sensor_msgs::image_encodings::BGR8;
            cv_bridge_img.header.frame_id = ros::this_node::getName();
            //first handle 02 color camera image
            if (!options.timestamps)
            {
                cv_bridge_img.header.stamp = current_timestamp ;
                ros_msg02.header.stamp = ros_cameraInfoMsg_camera02.header.stamp = cv_bridge_img.header.stamp;
            }


            cv_bridge_img.image = cv_image02;
            cv_bridge_img.toImageMsg(ros_msg02);

            // Publish raw image
            pub02.publish(ros_msg02, ros_cameraInfoMsg_camera02);

            // Publish image with bboxes
            publishImageWithBBoxes(raw_image_with_bboxes_pub, cv_image02,
                kitti_track_label->getObjectVec(entries_played), &cv_bridge_img.header);

            if(options.viewer) {
              // Add label drawing
              drawBBoxes(cv_image02, kitti_track_label->getObjectVec(entries_played));
            }

        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr points_pub;
        //publish velodyne lidar point cloud
        if (options.velodyne || options.all_data)
        {
            header_support.stamp = current_timestamp;
            full_filename_velodyne = dir_velodyne_points + sequence_num +"/" + boost::str(boost::format("%06d") % entries_played ) + ".bin";

            if (!options.timestamps)
              points_pub = publish_velodyne(map_pub, full_filename_velodyne, &header_support);
            double points_pub_timestamp = header_support.stamp.toSec();
        }


        static std::vector<std::string> lines; // every element represent a single line, don't change
        if(options.all_data ||options.gps || options.imu) {
          typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
          full_filename_oxts = dir_oxts + sequence_num + ".txt";

          static bool init_read = true;
          if(init_read) {
            // Read oxts contents in file
            boost::char_separator<char> sep_line {"\n"};

            // Read all contents in file
            std::ifstream t(full_filename_oxts);
            if (!t.is_open())
            {
              ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
              return 0;
            }

            ROS_DEBUG_STREAM("Reading GPS data from oxts file: " << full_filename_oxts );

            std::stringstream buffer;
            buffer << t.rdbuf();
            std::string contents(buffer.str());

            // Separate every line
            tokenizer tok_line(contents, sep_line);
            lines = std::vector<std::string>(tok_line.begin(), tok_line.end());

            init_read = false; // Dont repeat read
          }
        }

        //publish GPS data
        if (options.gps || options.all_data)
        {
          header_support.stamp = current_timestamp; //ros::Time::now();

          if (!getGPS(lines, entries_played, &ros_msgGpsFix, &header_support))
          {
            ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
            node.shutdown();
            return -1;
          }

          if (firstGpsData)
          {
            // this refs to BUG #551 - If a starting frame is specified, a wrong
            // initial-gps-fix is taken. Fixing this issue forcing filename to
            // 0000000001.txt
            // The FULL dataset should be always downloaded.
            if (!getGPS(lines, 1, &ros_msgGpsFix, &header_support))
            {
              ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
              node.shutdown();
              return -1;
            }
            ROS_DEBUG_STREAM("Setting initial GPS fix at " << endl << ros_msgGpsFix);
            firstGpsData = false;
            ros_msgGpsFixInitial = ros_msgGpsFix;
            ros_msgGpsFixInitial.header.frame_id = "/local_map";
            ros_msgGpsFixInitial.altitude = 0.0f;
          }

          gps_pub.publish(ros_msgGpsFix);
          gps_pub_initial.publish(ros_msgGpsFixInitial);
        }

        if (options.imu || options.all_data)
        {
          header_support.stamp = current_timestamp; //ros::Time::now();

          if (!getIMU(lines, entries_played, &ros_msgImu, &header_support))
          {
            ROS_ERROR_STREAM("Fail to open " << full_filename_oxts);
            node.shutdown();
            return -1;
          }
          imu_pub.publish(ros_msgImu);

        }

        // zhanghm: add for projection
        transform_matrix = project_matrix*R_rect_00*RT_velo_to_cam*transform_needed;

        points_pub = TransformKittiCloud(points_pub, true, 1.73);
        cv::Mat img_fusion_result = ProjectCloud2Image(points_pub, cv_image02, transform_matrix);
        cv::namedWindow("img_fusion_result", CV_WINDOW_NORMAL);
        cv::imshow("img_fusion_result", img_fusion_result);
        cv::waitKey(5);

        ++progress;
        entries_played++;

        if (!options.synchMode)
            loop_rate.sleep();
    }
    while (entries_played <= total_entries - 1 && ros::ok());


    if (options.viewer)
    {
        ROS_INFO_STREAM(" Closing CV viewer(s)");
        if (options.color || options.all_data)
            cv::destroyWindow("CameraSimulator Color Viewer");
        if (options.grayscale || options.all_data)
            cv::destroyWindow("CameraSimulator Grayscale Viewer");
        if (options.viewDisparities)
            cv::destroyWindow("Reprojection of Detected Lines");
        ROS_INFO_STREAM(" Closing CV viewer(s)... OK");
    }


    ROS_INFO_STREAM("Done!");
    node.shutdown();

    return 0;
}

