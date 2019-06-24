
#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "StructMovingTargetDefine.h"
#include <visualization/ShowResult.h>
#include "object_matching.h"

using namespace std;

class TargetMatching
{
public:
  TargetMatching();
  ~TargetMatching();
  void MapInput(int frame_counter_, double  time_stamp_, State_Vehicle ego_veh_state_,
                const std::vector<Line_int>& virtual_line_in);

  void MatchProcess( IplImage *img_ogm_, vector<CandidateObject>* candidate_object_vector,
      vector<MovingObject>* moving_object_vector_);

  void Init(float ogm_width_, float ogm_height_, float ogm_offest_y_,
            float ogm_resolution_,int virtual_line_num_, float virtual_line_resolution_,bool is_show_result = true);

  void Init(int map_width, int map_height, int map_offest_y,float map_resolution_,
            int virtual_line_num_, float virtual_line_resolution_,bool is_show_result = true);

private:
  void Release();
  /// Generating unique track ID for every tracked object
  int IDGenerator();

  bool Point_in_region(CvPoint2D32f point_, CvPoint2D32f* region_points_, int num_);

  int Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_ );

  float Cal_SimilarValue_first( CandidateObject candidate_object_, MovingObject moving_object_,
      Similar_Weight similar_weight_, int cal_method_);
  bool Cal_similar_traj(MovingObject moving_object_, CandidateObject candidate_object_);
  bool Cal_similar_rect(MovingObject moving_object_, CandidateObject candidate_object_);
  void Cal_MatchMatrix_first( vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_ );
  void MatchAnalsis( vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_,bool show_match = false);

  void ShowMatchMap(vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_);
  /*!
   * @brief 跟踪目标初始化,利用当前检测到的目标进行运动目标初始化,即新添加目标
   * @param moving_object_
   * @param candidate_object_
   */
  void Init_MovingObject(MovingObject *moving_object_, CandidateObject* candidate_object_);
  //利用匹配上的检测目标来更新当前运动目标的状态信息
  void Correct_ContourPoint4( MovingObject *moving_object_, CandidateObject* candidate_object_);
  void UpdateMeasurement(MovingObject& moving_object,const vector<CandidateObject>& candidate_object);
  void TrakedState( MovingObject *moving_object_ );
  /*!
   * @bried 根据跟踪次数等信息,更新跟踪状态,用来确定是否剔除跟踪目标以及提供其他信息
   * @param moving_object_
   */
  void UpdateTrackStatus(MovingObject *moving_object_);

  void UpdateMatchState( vector<CandidateObject>* candidate_object_vector, vector<MovingObject> *moving_object_vector_);
  /*!
   * @brief 初始化匹配参数,包括匹配状态等属性向量
   * @param moving_obj_num
   * @param candidate_obj_num
   */
  void InitMatchParam(int moving_obj_num,int candidate_obj_num);

private:
  int frame_counter;
  bool is_show_result_;
  State_Vehicle ego_veh_state_current;
  double  time_stamp;

  int candidate_object_num;
  int moving_object_num;

  int map_width;
  int map_height;
  int map_offset_y;
  float map_resolution;

  cv::Mat img_ogm_origin_;//原始输入栅格地图
  cv::Mat img_result_show_;//可视化用三通道图像
  IplImage *img_result;

  int* virtual_line_type;
  Line_int* boundary_line;
  int virtual_line_num;
  float virtual_line_resolution;

  ShowResult show_result;

  /// Object matching class for implementing object matching
  ObjectMatching tracker_global;

  TargetID* target_ID;
  int target_ID_index;//当前分配到的ID号

  std::vector<MatchState> match_state_mt_;
  std::vector<MatchState> match_state_cd_;
  MatchState init_matchstate_;
};
