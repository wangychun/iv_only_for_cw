#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

#include "StructMovingTargetDefine.h"
#include <visualization/ShowResult.h>
using namespace std;

class Tracker_local
{
public:
  Tracker_local();
  ~Tracker_local();

  void MapInput( int frame_counter_, float ogm_width_, float ogm_height_, float ogm_offest_y_, float ogm_resolution_,
      int virtual_line_num_, float virtual_line_resolution_, double  time_stamp_, State_Vehicle ego_veh_state_);

  void MatchProcess(vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject>* moving_object_vector_,
      Match_Satate* match_state_cd_, Match_Satate* match_state_mt_, float* similarvalue_matrix_,
      int target_num_cd_, int target_num_mt_);

  State_Vehicle ego_veh_state_current;
  double  time_stamp;
  int frame_counter;
protected:
  //
  void InitParam();
  void Release();

  //Init struct
  void Init_match_state(Match_Satate* match_state_, int index_size_);
  void Init_simlar_weight(Similar_Weight* similar_weight_, int index_size_);

  //
  bool Point_in_region(CvPoint2D32f point_, CvPoint2D32f* region_points_, int num_);
  //
  void Set_SimilarWeight_first(MovingObject moving_object_, int index_j_, Similar_Weight* similar_weight_, int match_step_);
  void Set_SimilarThershod_track( MovingObject moving_object_, int index_j_, float* similar_threshod_, int match_step_ );

  int Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_ );

  float Cal_SimilarValue_first( CandidateObject candidate_object_, MovingObject moving_object_,
      Similar_Weight similar_weight_, int cal_method_);
  bool Cal_similar_traj(MovingObject moving_object_, CandidateObject candidate_object_);
  bool Cal_similar_rect(MovingObject moving_object_, CandidateObject candidate_object_);
  // 3 match
  void Cal_MatchMatrix_first( vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject> *moving_object_vector_,
      Match_Satate* match_state_cd_, Match_Satate* match_state_mt_);
  void MatchAnalsis( vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject> *moving_object_vector_,
      Match_Satate* match_state_cd_, Match_Satate* match_state_mt_);

  int candidate_object_num;
  int moving_object_num;
  int match_matrix_size;
  float *similarvalue_matrix_cd;
  float *similarvalue_matrix_mt;


  float* similar_threshod_cd;
  Similar_Weight* similar_weight_cd;
  Match_Satate* match_state_cd;

  float* similar_threshod_mt;
  Similar_Weight* similar_weight_mt;
  Match_Satate* match_state_mt;

  int map_width;
  int map_height;
  int map_offset_y;
  float map_resolution;

  IplImage *img_ogm;
  IplImage *img_result;

  Line_int* boundary_line;
  int virtual_line_num;
  float virtual_line_resolution;

  float region_radius_resolution;
  float region_angle_resolution;
  float region_radius;
  float region_angle;
  int region_radius_cell;
  int region_angle_cell;
  int region_cell_size;
  Region_Target* region;


  int match_method;

  ShowResult show_result;

private:
};
