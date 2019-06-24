
#pragma once
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "StructMovingTargetDefine.h"
#include <visualization/ShowResult.h>
#include "TrajectoryDetection.h"
#include "Motion_global.h"

using namespace std;


class TrackingUpdate
{
public:
	TrackingUpdate();
	~TrackingUpdate();

	void Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
        const int& virtual_line_num_, const float& virtual_line_resolution_, bool is_show_result = true);

	void Init(int map_width, int map_height, int map_offest_y,float map_resolution,
              int virtual_line_num_, float virtual_line_resolution_,bool is_show_result = true);

	void MapInput(int frame_counter_,const double&  time_stamp_, const State_Vehicle& ego_veh_state_);

	void UpdateMotionState( IplImage *img_ogm_, vector<CandidateObject>* candidat_object_vector_,
							vector<MovingObject>* moving_object_vector_);

private:
	void Release();
	void Correct_ObjectShape( MovingObject *moving_object_, CandidateObject* candidate_object_ );
	void Identify_PoseHead( MovingObject *moving_object_);
	void ObjectTypeClassify(MovingObject *moving_object_);


	void MotionState_Filter_ab( MovingObject *moving_object_);
	void MotionState_Filter( MovingObject *moving_object_);
	void MotionState_Opt( MovingObject *moving_object_);
	void Motion_MissUpdate(MovingObject *moving_object_);
    void UpdateHistoryInfo(MovingObject* moving_object_);
	
	void MotionStateUpdating( vector<CandidateObject>* candidat_object_vector_, vector<MovingObject> *moving_object_vector_);
	void Target_Array( vector<MovingObject> *moving_object_vector_);
	MatchRange_grid Search_matchrange( IplImage* img_, MovingObject moving_object_);
	float Search_dis(MovingObject moving_object_, int index_);


private:
	double time_stamp;
	bool is_show_result_;

	int map_width;
	int map_height;
	int map_offset_y;
	float map_resolution;

    int    virtual_line_num;
    float virtual_line_resolution;

	State_Vehicle ego_veh_state_current;
	Motion_global motion_global;
    Traj_detect motion_traj;
    ShowResult show_result;

	CvPoint  ego_veh_position;
	Line_int* boundary_line;


	IplImage* img_ogm;
	IplImage* img_result;
};
