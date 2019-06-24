/*======================================================================
* Author   : Haiming Zhang
* Email    : zhanghm_1995@qq.com
* Version  :
* Copyright    :
* Descriptoin  : 二值栅格地图目标检测类
*                输入:二值栅格地图,车体坐标系下点云,完成目标聚类任务
*                明确输入输出,应该自定义输出结构体,符合检测结果需求,检测只是做检测,
*                封装性要强,其他转换关系不在本类完成
* References   :
======================================================================*/
#pragma once
//C++
#include <vector>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "StructMovingTargetDefine.h"

typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr CloudPtr;

struct ObjectInfo
{
  int numPoints;
  float length;
  float width;
  float height;
  float area;
  float LWratio;
  float HLratio;
  bool isPromising;

  ObjectInfo():
    numPoints(0),
    length(0.0),
    width(0.0),
    height(0.0),
    area(0.0),
    LWratio(0.0),
    HLratio(0.0),
    isPromising(false){}
};

class OGMDetector
{
public:
  //constructors
  OGMDetector(const OGMProperty& ogm_property, const PolarProperty& polar_property, bool is_show = true);
  ~OGMDetector();

  void SetInput(int frame_counter, const cv::Mat& img_ogm, const std::vector<OGM_Cell>& rigid_ogm, Polar_Cell* polar_ogm, const CloudPtr cloudIn);
  void SetInput(int frame_counter, const cv::Mat& img_ogm, const std::vector<OGM_Cell>& rigid_ogm, const vector<Polar_Cell>& polar_ogm, const CloudPtr cloudIn);
  void SetCloudInput(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn) { this->cloud_input_ = cloudIn;}
  //目标检测,对二值图像img_ogm_进行
  void Detect(vector<Line_int>& boundary_line);
  void DrawResult(bool draw_detect_info = true);
    /*!
  //目标检测,对二值图像img_ogm_进行
  void Detect(int frame_counter,const IplImage* const img_ogm_, const std::vector<OGM_Cell>& rigid_ogm_, Polar_Cell* polar_ogm_,
              vector<CandidateObject> *fusion_target_measure_, Line_int* boundary_line_);
  /*!
   * @brief 在二值图像中从指定点在指定范围内搜索栅格占据情况
   * @param img_binary
   * @param base_point
   * @param distance
   * @return
   */
  std::vector<Line_int> SearchOccupiedLine(const cv::Mat& img_binary,const cv::Point& base_point, float distance);

  //set and get functions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRGBClusteredCloud() const {return this->cloud_clustered_rgb_;}
  vector<CandidateObject> get_candidate_object_vector(bool do_transform = true);
  std::shared_ptr<vector<Cloud>> get_clustered_cloud_vec() const {return this->clusterd_cloud_vec_; }
private:
  //根据_img_ogm初始化和开辟必要内存
  bool AllocateMemories(const cv::Mat& img_ogm);
  cv::Mat RemoveIsolatedPixels(const cv::Mat& img_ogm);
  /*!
   * @brief according to clustered cloud vector and some rules to delete some object
   * @param clusterdCloud
   * @param total_object_vector
   * @param object_info_vec
   */
  void ruleBasedFilter(std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusterdCloud,
                       std::vector<CandidateObject>& total_object_vector,
                       std::vector<ObjectInfo>& object_info_vec);
  cv::Mat PreProcessing(const cv::Mat& img_input,bool is_show_result = false);

  void Virtual_line(const cv::Mat& binary_img);
  /*!
   * @brief 对二值图像进行轮廓查找聚类操作
   * @param img_binary
   * @param is_show
   */
  void TargetContourClustering(const cv::Mat& img_binary, bool is_show = false);
  void TargetIdentify( const std::vector<OGM_Cell>& rigid_ogm_, Polar_Cell* polar_ogm_);

  void Target_Array();

  /*!
   * @brief 将图像坐标系下点转为车体坐标系(x轴向右,y轴向前)下米制单位点坐标
   * @param object_image[in]  图像坐标系下点
   * @param object_vehicle[out] 车体坐标系下点
   */
  void TransfCandidate( const std::vector<CandidateObject>& object_image,std::vector<CandidateObject>& object_vehicle);

  void Release();

  void  InitLine(Line_s* line_, float x1_, float y1_, float x2_, float y2_);
  CvPoint2D32f  Point_line2(Line_s line1_, Line_s line2_);

  void ShowObject(IplImage* img, CandidateObject object_show_ , int object_ID_, CvScalar object_color_, int thinkness_);
  void InitCandidate( CvRect object_rect_, CvBox2D object_box_,CandidateObject* temp_object);

  void Grid_Points(CandidateObject* temp_object, vector<CvPoint2D32f> *grid_point_vector_,
      const std::vector<OGM_Cell>& rigid_ogm_, Polar_Cell* polar_ogm_);

  void Grid_fit_line(CandidateObject* temp_object,vector<CvPoint2D32f> grid_point_vector_,
      float* line_angle_, CvPoint2D32f* base_point_);
  void CorrectShape(CandidateObject* temp_object, vector<CvPoint2D32f> grid_point_vector_,
      float line_angle_, CvPoint2D32f base_point_);
  void OcclusionState(CandidateObject* temp_object, Polar_Cell* polar_ogm_);
  void Trackpoint_index(CandidateObject* temp_object, int step_);

  void TransformPolarPoint(CandidateObject* temp_object);
  /*!
   * @brief 目标类型判别
   * @param object_with_  宽度(米制单位）
   * @param object_length_ 长度(米制单位）
   * @return 1-9等级
   */
  int  ObjectType_Classify(const float& object_with_, const float& object_length_);
  void Identify_PoseHead( CandidateObject* temp_object);

  void Virtual_line_draw(Line_int* virtual_line_, int line_num_, float resolution_, const cv::Mat& img_, int method_);
  /*!
   * @brief 按相邻射线长度聚类射线
   * @param virtual_line_[in]
   * @param line_num_
   * @param line_cluster_vector_[out] 射线标号
   */
  void Virtual_line_cluster(Line_int* virtual_line_, int line_num_, vector<Cluster_Index>* line_cluster_vector_);

  void ClusteringVirtualLine(const vector<Line_int>& virtual_line_vector,vector<Cluster_Index>& line_cluster_vector);

  void getClusterdCloud(std::shared_ptr<std::vector<Cloud>> clusteredPoints,
                        const vector<CandidateObject>& total_object_vector,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,const std::vector<OGM_Cell>& rigid_ogm);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeClusteredCloud(const std::vector<Cloud>& cloudVec);

  void ShowDetectionResult(std::string window_name, cv::Mat& img_show, const vector<CandidateObject>& object_vector);

  /*!
   * @brief draw the detection results, including detected bouding boxes, IDs and some important info
   * @param img_show[in]           draw in which image
   * @param object_vector[in]      detected objects vector
   * @param draw_vehicle[optional] whether draw ego vehicle model
   */
  void DrawDetectionResult(cv::Mat& img_show, const vector<CandidateObject>& object_vector, bool draw_vehicle = true);
private:
  bool is_show_img;
  int frame_counter_;
  //Input contents
  cv::Mat img_ogm_input_;//输入的最原始栅格地图
  cv::Mat img_ogm_origin_;//保存经过预处理的单通道栅格地图
  cv::Mat img_color_ogm_;//原始三通道栅格地图

  CloudPtr cloud_input_;
  std::vector<OGM_Cell> rigid_ogm_vec_;
  std::vector<Polar_Cell> polar_ogm_vec_;
  Polar_Cell* polar_ogm_;

  vector<ObjectInfo> detect_objecs_info_;//聚类目标的信息
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clustered_rgb_;
  std::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZI>>> clusterd_cloud_vec_;

  cv::Mat img_temp_;//临时存储去除单个点栅格地图
  cv::Mat img_result_show_;//最终显示的图像,三通道图像

  int virtual_line_num;
  float virtual_line_resolution;
  Line_float* virtual_line_float;
  Line_int* virtual_line;
  Line_int* virtual_line_d;

  int* virtual_line_type;

  //map
  int map_offset_y;
  float map_resolution;
  CvPoint ego_veh_position;

  int polarogm_angle_cell;
  int polarogm_cell_size;

  vector<Cluster_Index> line_cluster_vector;
  vector<CandidateObject> total_object_vector;
};
