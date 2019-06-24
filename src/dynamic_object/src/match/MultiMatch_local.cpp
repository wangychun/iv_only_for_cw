#include "MultiMatch_local.h"
#include "../StructMovingTargetDefine.h"

Tracker_local::Tracker_local()
{
    frame_counter = 0;

    region_radius_resolution = 20;
    region_angle_resolution = 10;
    region_radius = 120;
    region_angle = 100;
    region_radius_cell = boost::math::round( region_radius / region_radius_resolution );
    region_angle_cell = boost::math::round( region_angle / region_angle_resolution );

}
Tracker_local::~Tracker_local()
{
}
//***************************  **************************//
bool Tracker_local::Point_in_region(CvPoint2D32f point_, CvPoint2D32f* region_points_, int num_)
{
    float temp_dot4[4];
    for (int m = 0; m < 4; ++m)
    {
        int m_up = BaseFunction::value_in_threshod_int(m+1, 0 , 3);
        temp_dot4[m] = (region_points_[m_up].x - region_points_[m].x)*(point_.y - region_points_[m].y)
                       - (region_points_[m_up].y - region_points_[m].y)*(point_.x - region_points_[m].x);
    }
    if( (temp_dot4[0] > 0 && temp_dot4[1] > 0 && temp_dot4[2] > 0 && temp_dot4[3] > 0)
        || (temp_dot4[0] < 0 && temp_dot4[1] < 0 && temp_dot4[2] < 0 && temp_dot4[3] < 0))
    {
        return true;
    }
    return false;
}
//******************** set match *******************//
void Tracker_local::Init_match_state(Match_Satate* match_state_, int index_size_)
{
    for (int i=0; i<index_size_; i++)
    {
        match_state_[i].is_match = false;
        match_state_[i].match_index = -1;
        match_state_[i].similar_num = 0;
        match_state_[i].similar_index_vector.clear();
        match_state_[i].max_similar_num = 0;
        match_state_[i].max_similar_index_vector.clear();
    }
}
void Tracker_local::Set_SimilarWeight_first(MovingObject moving_object_, int index_j_, Similar_Weight* similar_weight_, int match_step_)
{
    Similar_Weight similar_weight;
    similar_weight.similar_weight_x = 0.5;
    similar_weight.similar_weight_y = 0.5;

    similar_weight_[index_j_].similar_weight_x = similar_weight.similar_weight_x;
    similar_weight_[index_j_].similar_weight_y = similar_weight.similar_weight_y;
}
void Tracker_local::Set_SimilarThershod_track( MovingObject moving_object_, int index_j_, float* similar_threshod_, int match_step_ )
{
    float similar_value_threshod = 0.5;

    similar_threshod_[index_j_] = similar_value_threshod;
}
int Tracker_local::Match_ContourPoint4( MovingObject moving_object_, CandidateObject candidate_object_, int method_)
{
    if (moving_object_.object_type <2 || candidate_object_.object_type<2)
        return 0;

    CvPoint2D32f line4_cd[4]; // 01 12
    CvPoint2D32f line4_mt[4]; // 01 12 23 30
    if (method_ == 2)
    {
        for (int m=0; m<4; m++)
        {
            int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
            line4_cd[m].x = candidate_object_.shape.point4[m_up].x - candidate_object_.shape.point4[m].x;
            line4_cd[m].y = candidate_object_.shape.point4[m_up].y - candidate_object_.shape.point4[m].y;
            line4_mt[m].x = moving_object_.shape.point4[m_up].x - moving_object_.shape.point4[m].x;
            line4_mt[m].y = moving_object_.shape.point4[m_up].y - moving_object_.shape.point4[m].y;
        }
    }
    if (method_ == 1)
    {
        for (int m=0; m<4; m++)
        {
            int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
            line4_cd[m].x = candidate_object_.shape.point4[m_up].x - candidate_object_.shape.point4[m].x;
            line4_cd[m].y = candidate_object_.shape.point4[m_up].y - candidate_object_.shape.point4[m].y;
            line4_mt[m].x = moving_object_.shape.point4[m_up].x - moving_object_.shape.point4[m].x;
            line4_mt[m].y = moving_object_.shape.point4[m_up].y - moving_object_.shape.point4[m].y;
        }
    }

    int match_index0_cd = candidate_object_.track_index1;
    int match_index0_mt = moving_object_.track_index1;
    int line_index0_cd = match_index0_cd;
    float delta_angle_line0[4];
    float min_delta_angle0 = 361;
    for (int m=0; m<4; m++)
    {
        delta_angle_line0[m] = BaseFunction::Angle_line2(line4_cd[line_index0_cd], line4_mt[m]);
        if (delta_angle_line0[m] < min_delta_angle0)
        {
            min_delta_angle0 = delta_angle_line0[m];
            match_index0_mt = m;
        }
    }
    int match_index1_cd = BaseFunction::value_in_threshod_int(match_index0_cd+1, 0,3);
    int match_index1_mt = BaseFunction::value_in_threshod_int(match_index0_mt+1, 0,3);

    if (moving_object_.object_type >1 && moving_object_.object_type<5)
    {
        if (candidate_object_.object_type >1 && candidate_object_.object_type<5)
        {
            ;
        }
        if (candidate_object_.object_type > 4)
        {

        }
    }
    if (moving_object_.object_type >4)
    {
        if (candidate_object_.object_type >1 && candidate_object_.object_type<5)
        {
            ;
        }
        if (candidate_object_.object_type > 4)
        {
            ;
        }
    }

    int point_index_pre2cd = match_index0_cd - match_index0_mt;
    return point_index_pre2cd;
}
//******************** match detect and track *******************//
float Tracker_local::Cal_SimilarValue_first( CandidateObject candidate_object_, MovingObject moving_object_,
                                              Similar_Weight similar_weight_, int cal_method_)
{
    float similar_value = 0;

    int index0_cd = candidate_object_.shape.polar4[0].point_index;
    int index1_cd = candidate_object_.shape.polar4[1].point_index;
    int index2_cd = candidate_object_.shape.polar4[2].point_index;

    int index0_mt = moving_object_.shape.polar4[0].point_index;
    int index1_mt = moving_object_.shape.polar4[1].point_index;
    int index2_mt = moving_object_.shape.polar4[2].point_index;


    float dis_x_center = candidate_object_.center_point.x - moving_object_.center_point_pre.x;
    float dis_y_center = candidate_object_.center_point.y - moving_object_.center_point_pre.y;
    float dis_center = sqrt(dis_x_center*dis_x_center + dis_y_center*dis_y_center);

    CvPoint2D32f temp_point_cd = candidate_object_.center_point;
    if(candidate_object_.shape.polar_state == 2)
    {
        temp_point_cd.x = (candidate_object_.shape.point4[index1_cd].x + candidate_object_.shape.point4[index2_cd].x)/2;
        temp_point_cd.y = (candidate_object_.shape.point4[index1_cd].y + candidate_object_.shape.point4[index2_cd].y)/2;
    }

    bool is_region_radius = false;
    bool is_region_track = false;
    bool is_region_center = false;
    if (fabs(dis_x_center)<10 && fabs(dis_y_center)<20 )
    {
        is_region_track = Point_in_region(temp_point_cd, moving_object_.match_range.range_rect4, 4);
        is_region_center = Point_in_region(candidate_object_.center_point, moving_object_.match_range.range_rect4, 4);
        if ( dis_center <3 && moving_object_.object_type < 6)
            is_region_radius = true;
    }


    if (is_region_center || is_region_track || is_region_radius)//(dis_track < moving_object_.match_range.d_radius || dis_center < moving_object_.match_range.d_radius)
    {
        int point_index_pre2cd = 0;
        point_index_pre2cd = Match_ContourPoint4( moving_object_, candidate_object_, 1);


        Point2D track_point_cd = candidate_object_.shape.point4[index0_cd];
        Point2D track_point_mt = moving_object_.shape.point4[index0_mt];

        float delta_x = track_point_cd.x - track_point_mt.x;
        float delta_y = track_point_cd.y - track_point_mt.y;
        float dis_delta = sqrt(delta_x*delta_x + delta_y*delta_y);
        if (dis_center < dis_delta)
        {
            delta_x = dis_x_center;
            delta_y = dis_y_center;
        }


        //if(dis_delta < moving_object_.match_range.d_radius || dis_center < moving_object_.match_range.d_radius)
        {
            if (cal_method_ == 1)
            {
                float weight_x = 0.5;//similar_weight_.similar_weight_x;
                float weight_y = 0.5;//similar_weight_.similar_weight_y;
                similar_value = weight_x/(1 + fabs(delta_x)) + weight_y/(1 + fabs(delta_y));
            }
            if (cal_method_ == 2)
            {
                float weight_x = 0.5;//similar_weight_.similar_weight_x;
                float weight_y = 0.5;//similar_weight_.similar_weight_y;
                similar_value = weight_x/(1 + fabs(delta_x)) + weight_y/(1 + fabs(delta_y));
            }
        }
    }

    return similar_value;
}

bool Tracker_local::Cal_similar_traj(MovingObject moving_object_, CandidateObject candidate_object_)
{
    int object_type = abs(moving_object_.object_type);
    float delta_time = ego_veh_state_current.time_stamp - moving_object_.history_state.history_time[0];
    int point_index_pre2cd = Match_ContourPoint4( moving_object_, candidate_object_, 1);
    int index0 = moving_object_.shape.polar4[0].point_index;
    float delta_x = candidate_object_.center_point.x - moving_object_.center_point_pre.x;
    float delta_y = candidate_object_.center_point.y - moving_object_.center_point_pre.y;
    float temp_dis = sqrt(delta_x*delta_x + delta_y*delta_y);
    float temp_v_x = delta_x/delta_time;
    float temp_v_y = delta_y/delta_time;

    float ego_anlge = ego_veh_state_current.global_position.heading*pi/180;
    float rough_v_x = moving_object_.motion_state_ab.rough_v_x*cos(ego_anlge) - moving_object_.motion_state_ab.rough_v_y*sin(ego_anlge);
    float rough_v_y = moving_object_.motion_state_ab.rough_v_x*sin(ego_anlge) + moving_object_.motion_state_ab.rough_v_y*cos(ego_anlge);

    if (moving_object_.track_state.tracked_times<5 )
    {
        return false;
        if (temp_dis<5)
        {
            return true;
        } else
        {
            return false;
        }
//        if (object_type < 3 && temp_dis<2)
//        {
//            ;
//        }
//        if (object_type>2 && object_type <7)
//        {
//            ;
//        }
    }


    if (moving_object_.track_state.tracked_times>5)
    {
        float delta_v_x = temp_v_x - rough_v_x;
        float delta_v_y = temp_v_y - rough_v_y;
        float delta_v = sqrt(delta_v_x*delta_v_x + delta_v_y*delta_v_y);
        if (delta_v < 5)
        {
            return true;
        } else
        {
            return false;
        }
    }
    return false;
}
bool Tracker_local::Cal_similar_rect(MovingObject moving_object_, CandidateObject candidate_object_)
{
    bool is_false = false;
    int object_type_mt = abs(moving_object_.object_type);
    int object_type_cd = candidate_object_.object_type;
    CvPoint2D32f contour_point_pre[4];
    CvPoint2D32f center_pre = moving_object_.center_point_pre;
    CvPoint2D32f center_point_cd = candidate_object_.center_point;
    float delta_center_x = center_point_cd.x - center_pre.x;
    float delta_center_y = center_point_cd.y - center_pre.y;
    float dis_center = sqrt(delta_center_x*delta_center_x + delta_center_y*delta_center_y);
    if (dis_center<1.0)
    {
        return true;
    }
    for (int m = 0; m < 4; ++m)
    {
        contour_point_pre[m].x = moving_object_.shape.point4[m].x;
        contour_point_pre[m].y = moving_object_.shape.point4[m].y;
    }
    if (object_type_mt > 3)
    {
        is_false = Point_in_region(center_point_cd, contour_point_pre, 4);
        if (is_false)
            return is_false;
    }
    float temp_x = contour_point_pre[1].x - contour_point_pre[0].x;
    float temp_y = contour_point_pre[1].y - contour_point_pre[0].y;
    float temp_angle0 = atan2(temp_y, temp_x);
    float line_length0 = sqrt(temp_x*temp_x + temp_y*temp_y);
    float temp_angle1 = temp_angle0 + pi/2;
    temp_x = contour_point_pre[3].x - contour_point_pre[0].x;
    temp_y = contour_point_pre[3].y - contour_point_pre[0].y;
    float line_length1 = sqrt(temp_x*temp_x + temp_y*temp_y);
    float dis_line0 = (center_point_cd.x - center_pre.x)*sin(temp_angle0) - (center_point_cd.y - center_pre.y)*cos(temp_angle0);
    float dis_line1 = (center_point_cd.x - center_pre.x)*sin(temp_angle1) - (center_point_cd.y - center_pre.y)*cos(temp_angle1);
    float delta_dis1 = fabs(dis_line0) - line_length1;
    float delta_dis2 = fabs(dis_line1) - line_length0;
    if (delta_dis1<2 && delta_dis2<2)
        is_false = true;
    if(is_false)
        return false;

    return false;

}
void Tracker_local::Cal_MatchMatrix_first(vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject> *moving_object_vector_,
                                           Match_Satate* match_state_cd_, Match_Satate* match_state_mt_)
{
    int match_step = 1;
    //************** set match parament **************//
    for (int j=0; j<moving_object_vector_->size(); j++)
    {
        int index_j = j;
        MovingObject temp_moving_object = moving_object_vector_->at(j);
        Set_SimilarWeight_first(temp_moving_object, index_j, similar_weight_mt, match_step);
    }

    //************** calculate similar value **************//
    int cal_method_cd = 1;
    int cal_method_mt = 2;
    for (int j = 0; j < moving_object_num; j++)
    {
        int index_j = j;
        MovingObject temp_moving_object = moving_object_vector_->at(index_j);

        Similar_Weight temp_weight_mt = similar_weight_mt[j];
        for (int i=0; i<candidate_object_num; i++)
        {
            int matrix_index = j * candidate_object_num + i;
            CandidateObject temp_candidate = candidate_object_vevctor_->at(i);
            Similar_Weight temp_weight_cd = similar_weight_cd[i];

            float similar_value_mt = Cal_SimilarValue_first(temp_candidate, temp_moving_object, temp_weight_mt, cal_method_mt);
            float similar_value_cd = similar_value_mt;
            similarvalue_matrix_cd[matrix_index] = similar_value_cd;
            similarvalue_matrix_mt[matrix_index] = similar_value_mt;
            if(similar_value_mt > 0.1)
            {
                match_state_mt_[j].similar_num++;
                match_state_mt_[j].similar_index_vector.push_back(i);;
            }
            if(similar_value_cd > 0.1)
            {
                match_state_cd_[i].similar_num++;
                match_state_cd_[i].similar_index_vector.push_back(j);
            }
        }
    }

    //************** find max similar index **************//
    for (int j=0; j<moving_object_num; j++) //
    {
        float max_similar_value = 0;
        int index_i = -1;
        for (int i=0; i<match_state_mt_[j].similar_index_vector.size(); i++)
        {
            int index_cd = match_state_mt_[j].similar_index_vector.at(i);
            int matrix_index = j * candidate_object_num + index_cd;
            if ( similarvalue_matrix_mt[matrix_index] > max_similar_value )
            {
                max_similar_value = similarvalue_matrix_mt[matrix_index];
                index_i = index_cd;
            }
        }
        match_state_mt_[j].match_index = index_i;
        if (index_i > -1)
        {
            match_state_cd_[index_i].max_similar_num++;
            match_state_cd_[index_i].max_similar_index_vector.push_back(j);
        }
    }
    // tracked moving object max similar
    for (int i=0; i<candidate_object_num; i++) //
    {
        float max_similar_value = 0;
        int index_j = -1;
        for (int j=0; j<match_state_cd_[i].similar_index_vector.size(); j++)
        {
            int index_mt = match_state_cd_[i].similar_index_vector.at(j);
            int matrix_index = index_mt * candidate_object_num + i;
            if ( similarvalue_matrix_cd[matrix_index] > max_similar_value )
            {
                max_similar_value = similarvalue_matrix_cd[matrix_index];
                index_j = index_mt;
            }
        }
        match_state_cd_[i].match_index = index_j;
        if (index_j > -1)
        {
            match_state_mt_[index_j].max_similar_num++;
            match_state_mt_[index_j].max_similar_index_vector.push_back(i);
        }
    }

}

void Tracker_local::MatchAnalsis( vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject> *moving_object_vector_,
                                   Match_Satate* match_state_cd_, Match_Satate* match_state_mt_)
{
    Cal_MatchMatrix_first( candidate_object_vevctor_, moving_object_vector_, match_state_cd_, match_state_mt_ );
    match_method = 1;
    if (match_method == 2)//
    {
        for (int j=0; j<moving_object_num; j++)
        {
            int index_cd = match_state_mt_[j].match_index;
            if (index_cd > -1 )
            {
                int matrix_index = j * candidate_object_num + index_cd;
                if (similarvalue_matrix_mt[matrix_index] > 0.4
                    || (match_state_mt_[j].similar_num < 2 && match_state_mt_[j].similar_num >0) )
                {
                    match_state_mt_[j].is_match = true;
                    match_state_cd_[index_cd].is_match = true;
                    match_state_cd_[index_cd].match_index = j;
                }
            }
        }
        for (int i=0; i<candidate_object_num; i++)
        {
            if (match_state_cd_[i].match_index>-1)
            {
                match_state_cd_[i].is_match = true;
            }
        }
    }
    if (match_method == 1)
    {
        //************** match step 1, find maxium similar value match both **************//
        for (int j=0; j<moving_object_vector_->size(); j++)  //
        {
            MovingObject temp_object = moving_object_vector_->at(j);
            int object_type = abs(temp_object.object_type);
            if (match_state_mt_[j].similar_num > 0)
            {
                if (match_state_mt_[j].similar_num < 2)
                {
                    int index_cd = match_state_mt_[j].match_index;
                    int matrix_index = j*candidate_object_num + index_cd;
                    CandidateObject temp_candidate = candidate_object_vevctor_->at(index_cd);
                    if (match_state_cd_[index_cd].similar_num < 2)
                    {
                        bool is_false = false;
                        if(temp_object.is_updated)
                            is_false = true;
                        if (!is_false && object_type > 1)
                        {
                            is_false = Cal_similar_rect(temp_object, temp_candidate);
                        }
                        if (is_false || similarvalue_matrix_mt[matrix_index] > 0.75)
                        {
                            match_state_mt_[j].is_match = true;
                            match_state_cd_[index_cd].is_match = true;
                            match_state_cd_[index_cd].match_index = j;
                        }
                    }
                }
                if (match_state_mt_[j].similar_num > 1)
                {
                    if (temp_object.is_updated && temp_object.object_type>1)
                    {
                        int type_num = 0;
                        vector<int> part_vector;
                        for (int i = 0; i < match_state_mt_[j].similar_index_vector.size(); ++i)
                        {
                            int index_cd = match_state_mt_[j].similar_index_vector.at(i);
                            CandidateObject temp_candidate = candidate_object_vevctor_->at(index_cd);
                            bool is_false = Cal_similar_rect(temp_object, temp_candidate);


                            if (is_false)
                            {
                                type_num++;
                                part_vector.push_back(index_cd);
                                match_state_cd_[index_cd].is_match = true;
                                match_state_cd_[index_cd].match_index = j;
                            }
                        }
                        int max_index = -1;
                        float max_similar_value = 0;
                        for (int i = 0; i < part_vector.size(); ++i)
                        {
                            int index_cd = part_vector.at(i);
                            int matrix_index = j*candidate_object_num + index_cd;
                            if (similarvalue_matrix_mt[matrix_index] > max_similar_value)
                            {
                                max_index = index_cd;
                                max_similar_value = similarvalue_matrix_mt[matrix_index] ;
                            }
                        }
                        match_state_mt_[j].match_index = max_index;
                        if (max_index > -1)
                        {
                            match_state_mt_[j].is_match = true;
                            match_state_mt_[j].similar_num = 1;
                        } else
                        {
                            match_state_mt_[j].match_index = -1;
                            match_state_mt_[j].similar_index_vector.clear();
                        }
                    }
                }
            }
        }
        //************** match step 2, ensure the single similar match  **************//
        for (int i = 0; i < candidate_object_vevctor_->size(); i++)
        {
            CandidateObject temp_candidate = candidate_object_vevctor_->at(i);
            if (!match_state_cd_[i].is_match)
            {
                if (match_state_cd_[i].similar_num > 1)
                {
                    int type_num = 0;
                    vector<int> part_vector;
                    for (int j = 0; j < match_state_cd_[i].similar_index_vector.size(); ++j)
                    {
                        int index_mt = match_state_cd_[i].similar_index_vector.at(j);
                        MovingObject temp_object = moving_object_vector_->at(index_mt);

                        if (!match_state_mt_[index_mt].is_match)
                        {
                            type_num++;
                            part_vector.push_back(index_mt);
                        }
                    }
                    int max_index = -1;
                    int max_similar_value = 0;
                    for (int j = 0; j < part_vector.size(); ++j)
                    {
                        int index_mt = part_vector.at(j);
                        int matrix_index = index_mt*candidate_object_num + i;
                        if (similarvalue_matrix_mt[matrix_index] > max_similar_value)
                        {
                            max_index = index_mt;
                            max_similar_value = similarvalue_matrix_mt[matrix_index] ;
                        }
                    }
                    match_state_cd_[i].match_index = max_index;
                    if (max_index > -1)
                    {
                        match_state_cd_[i].is_match = true;
                        match_state_mt_[max_index].is_match = true;
                        match_state_mt_[max_index].match_index = i;
                    }
                }
            }
        }
    }
}

//************************* match input *************************//
void Tracker_local::InitParam()
{
    match_matrix_size = candidate_object_num * moving_object_num + 1 ;
    similarvalue_matrix_cd = new float[match_matrix_size];
    similarvalue_matrix_mt = new float[match_matrix_size];

    memset(similarvalue_matrix_cd, 0, match_matrix_size*sizeof(float));
    memset(similarvalue_matrix_mt, 0, match_matrix_size*sizeof(float));

    int temp_num_mt = moving_object_num + 1;
    int temp_num_cd = candidate_object_num + 1;

    similar_weight_cd = new Similar_Weight[temp_num_cd];
    memset(similar_weight_cd, 0.5 , temp_num_cd*sizeof( Similar_Weight) );

    similar_weight_mt = new Similar_Weight[temp_num_mt];
    memset(similar_weight_mt, 0.5, temp_num_mt*sizeof( Similar_Weight) );
}

void Tracker_local::Release()
{
    delete[] similar_weight_cd;
    delete[] similar_weight_mt;

    delete[] similarvalue_matrix_cd;
    delete[] similarvalue_matrix_mt;

//	cvReleaseImage(&img_ogm);
//	cvReleaseImage(&img_result);

}

void Tracker_local::MapInput( int frame_counter_, float ogm_width_, float ogm_height_, float ogm_offest_y_, float ogm_resolution_,
                               int virtual_line_num_, float virtual_line_resolution_, double  time_stamp_, State_Vehicle ego_veh_state_)
{
    frame_counter = frame_counter_;

    map_width = boost::math::round(ogm_width_ / ogm_resolution_);
    map_height = boost::math::round(ogm_height_ / ogm_resolution_);
    map_resolution = ogm_resolution_;
    map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);

    virtual_line_num = virtual_line_num_;
    virtual_line_resolution = virtual_line_resolution_;

    time_stamp_ = time_stamp_;

    ego_veh_state_current = ego_veh_state_;
}

void Tracker_local::MatchProcess(vector<CandidateObject>* candidate_object_vevctor_, vector<MovingObject>* moving_object_vector_,
                                  Match_Satate* match_state_cd_, Match_Satate* match_state_mt_, float* similarvalue_matrix_,
                                  int target_num_cd_, int target_num_mt_)
{
//    img_ogm = cvCreateImage(cvGetSize(img_ogm_), 8, 1);
//    img_result = cvCreateImage(cvGetSize(img_ogm_), 8, 3);
//
//    cvCopy(img_ogm_, img_ogm);
//    cvCvtColor(img_ogm_, img_result, CV_GRAY2BGR);

    candidate_object_num = target_num_cd_;
    moving_object_num = target_num_mt_;

    InitParam();

    memcpy(similarvalue_matrix_, similarvalue_matrix_mt, match_matrix_size*sizeof(float));
    if (candidate_object_vevctor_->size() >0 && moving_object_vector_->size()>0)
        MatchAnalsis( candidate_object_vevctor_, moving_object_vector_, match_state_cd_, match_state_mt_);

    Release();
}



