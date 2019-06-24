/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :
 * Copyright :
 * Descriptoin  : 数据关联类 匹配跟踪目标与当前检测目标
 *                1、目前实现最近邻匹配
 *                TODO:
 *                1、匈牙利算法多目标匹配
 *                2、JPDA匹配
 * References   :
======================================================================*/
#pragma once
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "StructMovingTargetDefine.h"
#include <visualization/ShowResult.h>
using namespace std;

class ObjectMatching
{
public:
  ObjectMatching();
  ~ObjectMatching();

  void Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
            const int& virtual_line_num_, const float& virtual_line_resolution_, bool is_show_result = true);

  void Init(int map_width, int map_height, int map_offest_y,float map_resolution,
            int virtual_line_num_, float virtual_line_resolution_,bool is_show_result = true);

  void ProcessMatch(vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject>* moving_object_vector_,
                    std::vector<MatchState>& match_state_cd,std::vector<MatchState>& match_state_mt, bool show_match = false);

  //set and get functions
  void set_img_ogm(cv::Mat img_in) {this->match_analysis_map_ = img_in;}
private:
  void InitParam();
  void CalcMatchMatrix(const vector<CandidateObject>* const candidate_object_vevctor_, const vector<MovingObject>* const moving_object_vector_);
  // Vector cross product to determine whether point in polygon
  bool Point_in_region(CvPoint2D32f point_, CvPoint2D32f* region_points_);
  int Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_ );
  /*!
   * @brief 计算跟踪目标和检测目标的相似度值, [0,1]取值范围,1-最相似
   * @param candidate_object
   * @param moving_object
   * @return
   */
  float CalcSimilarValue(CandidateObject candidate_object, MovingObject moving_object);

  void ShowMatchAnalysisMap(const vector<CandidateObject>& candidate_object_vevctor, const vector<MovingObject>& moving_object_vector);//绘制匹配分析图
private:
  bool is_show_result_;
  int candidate_object_num;
  int moving_object_num;

  int map_width;
  int map_height;
  int map_offset_y;
  float map_resolution;


  int virtual_line_num;
  float virtual_line_resolution;
  Line_int* boundary_line;


  cv::Mat match_analysis_map_;
  std::vector<MatchState> match_state_mt_;
  std::vector<MatchState> match_state_cd_;
  MatchState init_matchstate_;//初始化状态

  State_Vehicle ego_veh_state_current;


  Eigen::MatrixXf similar_value_matrix_;//相似度矩阵,行数为跟踪目标数,列数为当前检测目标数


  IplImage *img_ogm;
  IplImage *img_result;


};
