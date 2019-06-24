/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月1日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include <sensors_fusion/fusion_detection.h>
#include <pcl_ros/point_cloud.h>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <utils/ogm_mapping_utils.h>
#include <detect/box_fitting.h>

namespace sensors_fusion {

using namespace std;
using namespace cv;
using namespace sensor_msgs;

void createClusterdRGBCloud(const std::vector<ObjectTrack>& clusters_vec, pcl::PointCloud<pcl::PointXYZRGB>& rgb_cloud)
{
  int count = 1;
  for(const auto& obj : clusters_vec) {//遍历每个物体
//    rgb_cloud += obj.lidar_points;
    for(const auto& point : obj.lidar_points) {
      pcl::PointXYZRGB o;
      o.x = point.x;
      o.y = point.y;
      o.z = point.z;
      o.r = (500*count)%255;
      o.g = (100*count)%255;
      o.b = (150*count)%255;
      rgb_cloud.push_back(o);
    }
    ++count;
  }
  rgb_cloud.header.frame_id = "base_link";
}

void createRGBCloud(const pcl::PointCloud<pcl::PointXYZ>& cloudIn, pcl::PointCloud<pcl::PointXYZRGB>& cloudOut)
{
  static unsigned int count = 1;
  for(const auto& point : cloudIn) {
    pcl::PointXYZRGB o;
    o.x = point.x;
    o.y = point.y;
    o.z = point.z;
    o.r = (500*count)%255;
    o.g = (100*count)%255;
    o.b = (150*count)%255;
    cloudOut.push_back(o);
  }
  ++count;
}

FusionDetection::FusionDetection(ros::NodeHandle nh, ros::NodeHandle private_nh):
nh_(nh),
private_nh_(private_nh),
cloud_raw_(new VPointCloud),
cloud_limit_(new VPointCloud),
cloud_elevated_(new VPointCloud),
cloud_ground_(new VPointCloud),
cloud_camera_elevated_(new VPointCloud),
ogm_property_(OGMProperty(OGM_WIDTH, OGM_HEIGHT, OFFSET_Y, OGM_RESOLUTION)),
processthread_(NULL),
processthreadfinished_(false),
rviz_vis_(nh, private_nh),
projection_tools_(sensors_fusion::ProjectionSingleton::getInstance())
{
  // Set lidar camera transformation
  projection_tools_->init();
  Eigen::Matrix4f transform_kitti;
  transform_kitti<<0.0, 1.0, 0.0, 0.0,
                    -1.0,0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, -1.73,
                    0.0, 0.0, 0.0, 1.0;
  Eigen::MatrixXf transform_matrix(3,4);
  transform_matrix = projection_tools_->getTransformMatrix() * transform_kitti;
  projection_tools_->setTransformMatrix(transform_matrix);

  ogm_cells_vector_.resize(ogm_property_.mapCellNum());//TODO, why must need this???

  Size ogm_size = ogm_property_.mapSize();
  //可视化用
  vis_result_ = dynamic_object_tracking::VisualizerUtils::getInstance();
  vis_result_->Init(ogm_property_);

  // Define Publisher
  cloud_camera_elevate_pub_ = nh_.advertise<PointCloud2>(
      "/fusion_detection/cloud/camera_elelevated", 2);
  cloud_limit_pub_ = nh_.advertise<PointCloud2>(
      "/fusion_detection/cloud/cloud_limit",2 );
  cloud_elevated_pub_ = nh.advertise<PointCloud2>(
      "/fusion_detection/cloud/cloud_elevated",2 );
  cloud_clustered_rgb_pub_ = nh_.advertise<VRGBPointCloud>(
      "/fusion_detection/cloud/clustered_rgb", 2);

  image_detection_pub_ = nh_.advertise<Image>(
          "/fusion_detection/image/detection", 2);


  imageCloudSync_ = new MyMessagesSync(nh, "/darknet_ros/image_with_bboxes", "/kitti_player/hdl64e");

  // Begin process thread
  frame_count_ = 0;
  processthread_ = new boost::thread(boost::bind(&FusionDetection::process,this));
}

FusionDetection::~FusionDetection() {
  processthreadfinished_ = true;
  processthread_->join();
}

void FusionDetection::process()
{
  // main loop
  while(!processthreadfinished_&&ros::ok()) {
    // Get synchronized image with bboxes and cloud data
    sensors_fusion::MyMessagesSync::SyncImageCloudPair imagePair = imageCloudSync_->getSyncMessages();
    if((imagePair.first == nullptr) || (imagePair.second == nullptr)) {
      ROS_ERROR_THROTTLE(1,"Waiting for image and lidar topics!!!");
      continue;
    }

    ROS_WARN_STREAM("Begin process...");

    // Preprocess image and bounding bboxes
    processImage(imagePair.first);

    // Preprocess point cloud
    processPointCloud(imagePair.second, false);

    // Process cluster
    processCluster(cloud_camera_elevated_);


    // Box fitting
    boxFitting();

    // Print
    for(const auto& obj : clusters_track_) {
      cout<<obj.bbox3D.size()<<endl;
    }

    // Visualization
    rviz_vis_.showBoundingBoxes(clusters_track_);
  }
}

void FusionDetection::processImage(const darknet_ros_msgs::ImageWithBBoxes::ConstPtr& image)
{
  // Get image
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image->image,  sensor_msgs::image_encodings::BGR8);
  image_raw_ = cv_ptr->image;

  // Get bounding boxes
  detectBBoxes_ = darknetMsgToBBoxes(image);

  // Draw bounding boxes
  drawBBoxes(image_raw_, detectBBoxes_);
//  namedWindow("image_raw", CV_WINDOW_NORMAL);
//  imshow("image_raw", image_raw_);
//  waitKey(10);

  // Publish detection grid
  cv_bridge::CvImage cv_detection_grid_image;
  cv_detection_grid_image.image = image_raw_;
  cv_detection_grid_image.encoding = image_encodings::BGR8;
  cv_detection_grid_image.header.stamp = image->header.stamp;
  image_detection_pub_.publish(cv_detection_grid_image.toImageMsg());
}

void FusionDetection::processPointCloud(const sensor_msgs::PointCloud2::ConstPtr & cloud, bool isShow)
{
  VPointCloud::Ptr tempcloud(new VPointCloud());//当前帧点云（雷达里程计坐标系）
  pcl::fromROSMsg(*cloud, *tempcloud);

  // 将kitti点云转换到本车常用车体坐标系下
  sensors_fusion::TransformKittiCloud(*tempcloud, *cloud_raw_, true, 1.73);
  cloud_raw_->header.frame_id = "base_link"; // Vehicle coordinate

  // Filter cloud
  float x_limit = ogm_property_.ogm_width_/2.0;
  float y_forward_limit = ogm_property_.ogmYForwardLim();
  float y_backward_limit = ogm_property_.ogmYBackwardLim();
  sensors_fusion::CloudFilter(*cloud_raw_, *cloud_limit_, -x_limit, x_limit, y_backward_limit, y_forward_limit, -0.5, 3.5);

  // Publish filtered cloud
  cloud_limit_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
  cloud_limit_pub_.publish(cloud_limit_);

  /// Ground cloud removal
  sensors_fusion::ogm_mapping::CreateOGM(*cloud_limit_, this->ogm_cells_vector_, this->ogm_property_); // Create ogm vector
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ExtractIndices<VPoint> pcl_extractor;
  sensors_fusion::RemoveGround(*cloud_limit_, inliers, this->ogm_property_, ogm_cells_vector_, 0.5);

  // Clear the  cloud
  cloud_elevated_->points.clear();
  cloud_ground_->points.clear();

  // Extract points from original point cloud
  pcl_extractor.setInputCloud(cloud_limit_);
  pcl_extractor.setIndices(inliers);
  pcl_extractor.setNegative(false);
  pcl_extractor.filter(*cloud_elevated_);

  pcl_extractor.setInputCloud(cloud_limit_);
  pcl_extractor.setIndices(inliers);
  pcl_extractor.setNegative(true);
  pcl_extractor.filter(*cloud_ground_);

  cloud_elevated_->header.stamp = pcl_conversions::toPCL(cloud->header.stamp);
  cloud_elevated_->header.frame_id = "base_link";
  cloud_elevated_pub_.publish(cloud_elevated_);

  if(isShow) {//可视化移除地面点云
    cv::Mat ground_removal_img = vis_result_->DrawPointCloudOGM(*cloud_elevated_);
    cv::namedWindow("ground_removal_img",CV_WINDOW_NORMAL);
    cv::imshow("ground_removal_img",ground_removal_img);
    cv::waitKey(5);
  }


  /// Extract elevated points in camera view
  // Clear the cloud
  cloud_camera_elevated_->points.clear();
  for(int i = 0; i < cloud_elevated_->size(); ++i) {
    VPoint& point = cloud_elevated_->at(i);
    if(point.y < 0)
      continue;
    Point2f point2d;
    sensors_fusion::ProjectPoint2Image(point, projection_tools_->getTransformMatrix(), point2d);

    // Check if image point is valid
    const int& img_x = point2d.x;
    const int& img_y = point2d.y;

    if((img_x >= 0 && img_x < image_raw_.cols)&&
        (img_y >= 0 && img_y < image_raw_.rows)) {
      cloud_camera_elevated_->points.push_back(point);
    }
  }

  // Publish elevate cloud in camera view
  cloud_camera_elevated_->header= cloud_raw_->header;
  cloud_camera_elevate_pub_.publish(cloud_camera_elevated_);
}

void FusionDetection::processCluster(VPointCloud::Ptr cloudIn)
{
//  clusters_track_.clear();
  clusters_track_ = vector<ObjectTrack>(detectBBoxes_.size());

  // 1) Loop for every point in cloud
  for(const auto& point : *cloudIn) {
    if(point.y < 0)
      continue;
    cv::Point2f imgPoint;
    ObjectTrack t;
    ProjectPoint2Image(point, projection_tools_->getTransformMatrix(), imgPoint);

    //2) Loop every bbox in input bboxes
    for(int i = 0; i < detectBBoxes_.size(); ++i) {
      if(detectBBoxes_[i].bbox_.contains(imgPoint)) {
        VPoint p;
        p.x = point.x;
        p.y = point.y;
        p.z = point.z;
//        p.r = color[detectBBoxes_[i].className_].val[2];
//        p.g = color[detectBBoxes_[i].className_].val[1];
//        p.b = color[detectBBoxes_[i].className_].val[0];
        clusters_track_[i].lidar_points.push_back(p);
        // TODO: will repeat assign, should change
        clusters_track_[i].bbox = detectBBoxes_[i].bbox_;
        clusters_track_[i].object_type = detectBBoxes_[i].className_;
      }
    }
  }

  ROS_INFO_STREAM("======"<<clusters_track_.size());

  // Using euclidean cluster for removing background in every bboxes
  removeClustersBackground();

  // Publish clustered cloud
  VRGBPointCloud cloud_rgb_clustered;
  createClusterdRGBCloud(clusters_track_,cloud_rgb_clustered);
  cloud_clustered_rgb_pub_.publish(cloud_rgb_clustered);
}

void FusionDetection::boxFitting()
{
  // Construct clustered object vector
  vector<VPointCloud>  clusteredVec;
  for(const auto& c : clusters_track_) {
    ROS_WARN_STREAM("boxFitting "<< c.lidar_points.size());
    clusteredVec.push_back(c.lidar_points);
  }

  // Get 3D bounding boxes
  vector<VPointCloud>  bbPoints(clusters_track_.size()); // every 3D bbox has 8 vertices
  getBoundingBox(clusteredVec, bbPoints);

  int t = 0;
  for(const auto& bb : bbPoints) {
    clusters_track_[t].bbox3D = bb;
    if(bb.size() == 0)
      clusters_track_[t].is_valid = false;
    else
      clusters_track_[t].is_valid = true;
    ++t;
  }
}

void FusionDetection::removeClustersBackground()
{
  // initialize cluster_indices
  std::vector<std::vector<pcl::PointIndices> > cluster_indices =
      std::vector<std::vector<pcl::PointIndices> >(clusters_track_.size());

  // Loop every object
  for(int i = 0; i < clusters_track_.size(); ++i) {
    VPointCloud& cloud_point = clusters_track_[i].lidar_points;

    cout<<"euclidean clustering "<<i<<" has number "<< cloud_point.size()<<" "<<
        clusters_track_[i].empty()<<endl;

    // Check cloud point size
    if(cloud_point.size() <= 10) {
      clusters_track_[i].is_valid = false;
      continue;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<VPoint>::Ptr tree (new pcl::search::KdTree<VPoint>);
    tree->setInputCloud (cloud_point.makeShared()); //创建点云索引向量，用于存储实际的点云信息

    pcl::EuclideanClusterExtraction<VPoint> ec;
    ec.setClusterTolerance (0.2); //设置近邻搜索的搜索半径为20cm
    ec.setMinClusterSize (30);//设置一个聚类需要的最少点数目为100
    ec.setMaxClusterSize (2500);//设置一个聚类需要的最大点数目为25000
    ec.setSearchMethod (tree);//设置点云的搜索机制
    ec.setInputCloud (cloud_point.makeShared());
    ec.extract (cluster_indices[i]);//从点云中提取聚类，并将点云索引保存在cluster_indices中
  }

  // Loop every object clusters indices
  for(int i = 0; i < cluster_indices.size(); ++i) {
    auto& clusters = cluster_indices[i];

    if(clusters.size() == 0)
      continue;

    // Loop every euclidean subset cluster in a object
    std::vector<VPointCloud::Ptr> cloud_clusters_vec;
    for(auto it = clusters.begin(); it != clusters.end(); ++it) {
      VPointCloud::Ptr cloud_cluster (new VPointCloud);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        cloud_cluster->points.push_back (clusters_track_[i].lidar_points.points[*pit]);
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      cloud_clusters_vec.push_back(cloud_cluster);
    }
    cout<<"--------"<<i<<"cloud_clusters_vec size is "<<cloud_clusters_vec.size()<<endl;

    // Find most close one
    auto it_min_dis = std::min_element(cloud_clusters_vec.begin(), cloud_clusters_vec.end(),
        [](const VPointCloud::Ptr lhs, const VPointCloud::Ptr rhs) {
      return calculateAverageDistance(*lhs) < calculateAverageDistance(*rhs);
    });
    if(it_min_dis != cloud_clusters_vec.end()) {
      clusters_track_[i].lidar_points = *(*it_min_dis);
    }
  }

}


vector<sensors_fusion::BBox2DBase> FusionDetection::darknetMsgToBBoxes(const darknet_ros_msgs::ImageWithBBoxesConstPtr& darknetMsg)
{
  vector<sensors_fusion::BBox2DBase> res;
  vector<darknet_ros_msgs::BoundingBox> bboxVec = darknetMsg->bboxes.bounding_boxes;
  for(const auto& box : bboxVec) {
    if(box.Class == "Car" || box.Class == "Pedestrian" || box.Class == "Cyclist") {
      Rect rect(cv::Point(box.xmin, box.ymin), cv::Point(box.xmax, box.ymax));
      sensors_fusion::BBox2DBase boxBase(rect, box.Class, box.probability);
      res.push_back(boxBase);
    }
  }
  return res;
}

void FusionDetection::drawBBoxes(cv::Mat& image, const vector<BBox2DBase>& bbox)
{
  auto getRectCenter = [](const cv::Rect& rect)->cv::Point{
    return cv::Point(cvRound(rect.x + rect.width/2.0), cvRound(rect.y + rect.height/2));
  };
  for(const auto& box : bbox) {
    if(box.className_ == "Car") {
      rectangle(image, box.bbox_, color["Car"], 2); // blue
      circle(image, getRectCenter(box.bbox_), 2, color["Car"], 2);
    }
    else if(box.className_ == "Pedestrian") {
      rectangle(image, box.bbox_, color["Pedestrian"], 2); // green
      circle(image, getRectCenter(box.bbox_), 2, color["Pedestrian"], 2);
    }
    else if(box.className_ == "Cyclist") {
      cv::rectangle(image, box.bbox_, color["Cyclist"], 2);
      circle(image, getRectCenter(box.bbox_), 2, color["Cyclist"], 2);
    }
    else {
      cv::rectangle(image, box.bbox_, cv::Scalar(255, 255, 255), 2);
      circle(image, getRectCenter(box.bbox_), 2, cv::Scalar(255, 255, 255), 2);
    }
  }
}

} /* namespace sensors_fusion */
