#include "Motion_global.h"
Motion_global::Motion_global()
{
}
Motion_global::~Motion_global()
{
}

void Motion_global::Init(const float& ogm_width_, const float& ogm_height_, const float& ogm_offest_y_, const float& ogm_resolution_,
    const int& virtual_line_num_, const float& virtual_line_resolution_)
{
   map_width = boost::math::round(ogm_width_ / ogm_resolution_) +1;
   map_height = boost::math::round(ogm_height_ / ogm_resolution_) +1;
   map_resolution = ogm_resolution_;
   map_offset_y = boost::math::round(ogm_offest_y_ / map_resolution);
}

void Motion_global::MapInput(int frame_counter_,double  time_stamp_, State_Vehicle ego_veh_state_)
{
  frame_counter = frame_counter_;
  time_stamp = time_stamp_;
  ego_veh_state_current = ego_veh_state_;
}

void Motion_global::FilterMethold_KF_PosHead_ab( MovingObject *moving_object_)
{
  float head_index = moving_object_->shape.head_index;
  if(head_index<0)
  {
    head_index = 0;
  }
  float pos_head_measure = moving_object_->pose_head + ego_veh_state_current.global_position.heading*pi/180;
  if(moving_object_->filter_pos_head_ab > 0)
  {
    float delta_angle = pos_head_measure - moving_object_->motion_state_ab.pose_head_post;
    if(delta_angle > pi)
    {
      pos_head_measure = pos_head_measure - 2*pi;
    }
    if(delta_angle < -pi)
    {
      pos_head_measure = pos_head_measure + 2*pi;
    }
  }
  int filter_state = moving_object_->filter_pos_head_ab;
  int delta_times = 2;
  if (moving_object_->filter_pos_head_ab == (delta_times+1))
  {
    moving_object_->filter_pos_head_ab = 1;
  }
  switch (moving_object_->filter_pos_head_ab)
  {
  case 0://init KF kalman filter
  {
    moving_object_->kalman_pos_head_ab = cvCreateKalman(2,1,0);
    const float F[]=    //³õÊŒ»¯Ž«µÝŸØÕó
    {  1,0.1*delta_times,
        0,1
    };
    memcpy( moving_object_->kalman_pos_head_ab->transition_matrix->data.fl, F, sizeof(F) );

    cvSetIdentity( moving_object_->kalman_pos_head_ab->measurement_matrix, cvScalarAll(1) );

    const float P_head_KF = (6.5*pi/180);//200
    const float P_v_w_KF = (6.5*pi/180);//200

    const float Q_head_KF = 0.05;//200
    const float Q_v_w_KF = 0.05;//200

    const float R_head_KF = 0.5;//5

    const float Q[]=
    {
        pow(Q_head_KF,2),0,
        0,pow(Q_v_w_KF,2)
    };
    memcpy(moving_object_->kalman_pos_head_ab->process_noise_cov->data.fl,Q, sizeof(Q));

    const float R[]=
    {   pow(R_head_KF,2)
    };
    memcpy(moving_object_->kalman_pos_head_ab->measurement_noise_cov->data.fl, R, sizeof(R));

    const float P[]=
    {
        pow(P_head_KF,2),0,
        0,pow(P_v_w_KF,2)
    };
    memcpy( moving_object_->kalman_pos_head_ab->error_cov_post->data.fl, P, sizeof(P) );

    CvMat* measurement_matrix = cvCreateMat(1, 1, CV_32FC1);//²âÁ¿ŸØÕó2Î¬£¬x¡¢y×ø±ê

    moving_object_->kalman_pos_head_ab->state_post->data.fl[0] = pos_head_measure;
    moving_object_->kalman_pos_head_ab->state_post->data.fl[1] = 0;

    cvKalmanPredict(moving_object_->kalman_pos_head_ab, 0);
    moving_object_->motion_state_ab.pose_head_pred= moving_object_->kalman_pos_head_ab->state_pre->data.fl[0];
    moving_object_->motion_state_ab.v_w_pred = moving_object_->kalman_pos_head_ab->state_pre->data.fl[1];

    measurement_matrix->data.fl[0] = pos_head_measure;

    cvKalmanCorrect( moving_object_->kalman_pos_head_ab, measurement_matrix);
    cvReleaseMat( &measurement_matrix );

    moving_object_->motion_state_ab.pose_head_post = moving_object_->kalman_pos_head_ab->state_post->data.fl[0];
    moving_object_->motion_state_ab.v_w_post = moving_object_->kalman_pos_head_ab->state_post->data.fl[1];
  }break;
  case 1://filter
  {
    cvKalmanPredict(moving_object_->kalman_pos_head_ab, 0);

    moving_object_->motion_state_ab.pose_head_pred = moving_object_->kalman_pos_head_ab->state_pre->data.fl[0];
    moving_object_->motion_state_ab.v_w_pred = moving_object_->kalman_pos_head_ab->state_pre->data.fl[1];

    CvMat* measurement_matrix = cvCreateMat(1, 1, CV_32FC1);//²âÁ¿ŸØÕó2Î¬£¬x¡¢y×ø±ê
    if(moving_object_->is_updated)
    {
      measurement_matrix->data.fl[0] = pos_head_measure;
    }
    else
    {
      measurement_matrix->data.fl[0] = moving_object_->kalman_pos_head_ab->state_pre->data.fl[0];
    }


    cvKalmanCorrect( moving_object_->kalman_pos_head_ab, measurement_matrix);
    cvReleaseMat( &measurement_matrix );

    moving_object_->motion_state_ab.pose_head_post = moving_object_->kalman_pos_head_ab->state_post->data.fl[0];
    moving_object_->motion_state_ab.v_w_post = moving_object_->kalman_pos_head_ab->state_post->data.fl[1];

  }break;
  default: break;
  }
  if (moving_object_->filter_pos_head_ab > 0)
  {
    moving_object_->filter_pos_head_ab++;
  }
  if (moving_object_->filter_pos_head_ab == 0)
  {
    moving_object_->filter_pos_head_ab = 1;
  }
}

void Motion_global::FilterMethold_KF_Center2_ab( MovingObject *moving_object_)
{
  double px,py;
  //利用目标中心点作为观测量进行更新,考虑到全局坐标数值太大,转到以刚获取到该目标时本车的全局坐标位置为准
  //  px = moving_object_->center_point_ab.x;
  //  py = moving_object_->center_point_ab.y;
  px = moving_object_->center_pt_meas_ab.x;
  py = moving_object_->center_pt_meas_ab.y;
//  cout<<"pxxxxxxxxxxxxxxxxxxxxxxx "<<px<<" "<<py<<endl;
//  cout<<"measurementttttttttttttt "<<moving_object_->center_pt_meas_ab.x<<" "<<moving_object_->center_pt_meas_ab.y<<endl;

  if(moving_object_->is_new_add){//Kalman滤波初始化
    moving_object_->KF_center_ab = cv::KalmanFilter(4,2,0,CV_64F);
    moving_object_->KF_center_ab.statePost = (cv::Mat_<double>(4,1)<<px,py,0,0);//初始状态 x
    moving_object_->KF_center_ab.transitionMatrix = (cv::Mat_<double>(4,4)
        <<1,0,0.1,0,
        0,1,0,0.1,
        0,0,1,0,
        0,0,0,1);

    const float P_x_KF = 1;//200
    const float P_y_KF = 1;//200
    const float P_v_x_KF = 100;//70.7
    const float P_v_y_KF = 100;//70.7

    const float Q_x_KF = 0.05;//5
    const float Q_y_KF = 0.05;//5
    const float Q_v_x_KF = 0.05;//7.07
    const float Q_v_y_KF = 0.05;//7.07
    const double Q_var = 0.5;

    const float R_x_KF = 0.25;
    const float R_y_KF = 0.25;

    moving_object_->KF_center_ab.errorCovPost = (cv::Mat_<double>(4,4)
        <<pow(P_x_KF,2),0,0,0,
        0,pow(P_y_KF,2),0,0,
        0,0,pow(P_v_x_KF,2),0,
        0,0,0,pow(P_v_y_KF,2));//后验错误协方差P
    //    moving_object_->KF_center_ab.processNoiseCov = (cv::Mat_<double>(4,4)
    //        <<pow(Q_x_KF,2),0,0,0,
    //        0,pow(Q_y_KF,2),0,0,
    //        0,0,pow(Q_v_x_KF,2),0,
    //        0,0,0,pow(Q_v_y_KF,2));//过程噪声协方差Q
    moving_object_->KF_center_ab.processNoiseCov = pow(Q_var,2)*(cv::Mat_<double>(4,4)
        <<pow(0.1,2)*pow(0.1,2)/4,0,0.1*0.1*0.1/2,0,
        0,pow(0.1,2)*pow(0.1,2)/4,0,0.1*0.1*0.1/2,
        0.1*0.1*0.1/2,0,0.1*0.1,0,
        0,0.1*0.1*0.1/2,0,0.1*0.1);//过程噪声协方差Q
    moving_object_->KF_center_ab.measurementNoiseCov = (cv::Mat_<double>(2,2)
        <<pow(R_x_KF,2),0,
        0,pow(R_y_KF,2));//测量噪声协方差R
    moving_object_->KF_center_ab.measurementMatrix = (cv::Mat_<double>(2,4)
        <<1,0,0,0,
        0,1,0,0);//观测矩阵H

    //    moving_object_->is_KF_center_ab_initialized = true;

    moving_object_->motion_state_ab.x_post = moving_object_->KF_center_ab.statePost.at<double>(0);
    moving_object_->motion_state_ab.y_post = moving_object_->KF_center_ab.statePost.at<double>(1);
    moving_object_->motion_state_ab.v_x_post = moving_object_->KF_center_ab.statePost.at<double>(2);
    moving_object_->motion_state_ab.v_y_post = moving_object_->KF_center_ab.statePost.at<double>(3);
    moving_object_->motion_state_ab.v_post = (float)sqrt(moving_object_->motion_state_ab.v_x_post * moving_object_->motion_state_ab.v_x_post
        + moving_object_->motion_state_ab.v_y_post * moving_object_->motion_state_ab.v_y_post);
    moving_object_->motion_state_ab.v_theta_post = (float)atan2(moving_object_->motion_state_ab.v_y_post, moving_object_->motion_state_ab.v_x_post);
  }
  else{
    //1)Prediction
    //      时间间隔大于0.1s,则多次预测
    //      while (delta_t > 0.1){
    //        const double dt = 0.05;
    //        moving_object_->KF_center_ab.transitionMatrix = (cv::Mat_<double>(4,4) //状态转移矩阵F
    //            <<1,0,dt,0,
    //            0,1,0,dt,
    //            0,0,1,0,
    //            0,0,0,1);
    //        moving_object_->KF_center_ab.predict();
    //        delta_t -= dt;
    //      }
    //      moving_object_->KF_center_ab.transitionMatrix = (cv::Mat_<double>(4,4)
    //          <<1,0,delta_t,0,
    //          0,1,0,delta_t,
    //          0,0,1,0,
    //          0,0,0,1);

    moving_object_->KF_center_ab.predict();

    //2)Update
    cv::Mat measurement = cv::Mat::zeros(2,1,CV_64F);
    if (moving_object_->has_match){//有匹配目标,此时px,py为当前检测到匹配目标的更新值
      measurement.at<double>(0) = px;
      measurement.at<double>(1) = py;
    }
    else{//否则,默认与上一时刻测量位置相同
      measurement.at<double>(0) = moving_object_->KF_center_ab.statePost.at<double>(0);
      measurement.at<double>(1) = moving_object_->KF_center_ab.statePost.at<double>(1);
    }
    moving_object_->KF_center_ab.correct(measurement);

    //更新跟踪信息
    moving_object_->motion_state_ab.x_post = moving_object_->KF_center_ab.statePost.at<double>(0);
    moving_object_->motion_state_ab.y_post = moving_object_->KF_center_ab.statePost.at<double>(1);
    moving_object_->motion_state_ab.v_x_post = moving_object_->KF_center_ab.statePost.at<double>(2);
    moving_object_->motion_state_ab.v_y_post = moving_object_->KF_center_ab.statePost.at<double>(3);
    moving_object_->motion_state_ab.v_post = (float)sqrt(moving_object_->motion_state_ab.v_x_post * moving_object_->motion_state_ab.v_x_post
        + moving_object_->motion_state_ab.v_y_post * moving_object_->motion_state_ab.v_y_post);
    moving_object_->motion_state_ab.v_theta_post = (float)atan2(moving_object_->motion_state_ab.v_y_post, moving_object_->motion_state_ab.v_x_post);

    if(!moving_object_->has_match){//没有匹配,用Kalman估计值更新测量值,作为目标丢失时的最优估计,用于加到历史信息中
      moving_object_->center_pt_meas_ab.x = moving_object_->motion_state_ab.x_post;
      moving_object_->center_pt_meas_ab.y = moving_object_->motion_state_ab.y_post;
    }
  }//end else
//  float temp_v = sqrt(pow(moving_object_->KF_center_ab.statePost.at<double>(2),2) + pow(moving_object_->KF_center_ab.statePost.at<double>(3),2));
}

void Motion_global::FilterUpdate_Pt4_ab( MovingObject *moving_object_)
{
  //更新矩形框四个点坐标
  for (int i=0; i<4; i++){
//    CvPoint2D32f position_measure = moving_object_->track_point;
    CvPoint2D32f position_measure;
    position_measure.x = moving_object_->contour_rect_ab.point4[i].x
        - moving_object_->ego_veh_start.global_position.dLng;
    position_measure.y = moving_object_->contour_rect_ab.point4[i].y
        - moving_object_->ego_veh_start.global_position.dLat;

    if (!moving_object_->is_KF_pt4_ab_initialized){
      //init KF kalman filter
      moving_object_->kalman_pt4_ab[i] = cvCreateKalman(6,2,0);
      const float F[]=
      { 1,0,0.2,0,0.02,0,
          0,1,0,0.2,0,0.02,
          0,0,1,0,0.2,0,
          0,0,0,1,0,0.2,
          0,0,0,0,1,0,
          0,0,0,0,0,1
      };

      memcpy( moving_object_->kalman_pt4_ab[i]->transition_matrix->data.fl, F, sizeof(F) );

      cvSetIdentity( moving_object_->kalman_pt4_ab[i]->measurement_matrix, cvScalarAll(1) );

      const float P_x_KF = 10;//200
      const float P_y_KF = 10;//200
      const float P_v_x_KF = 2.07;//70.7
      const float P_v_y_KF = 2.07;//70.7
      const float P_a_x_KF = 1.07;//70.7
      const float P_a_y_KF = 1.07;//70.7

      const float Q_x_KF = 0.05;//5
      const float Q_y_KF = 0.05;//5
      const float Q_v_x_KF = 0.0707;//7.07
      const float Q_v_y_KF = 0.0707;//7.07
      const float Q_a_x_KF = 0.0707;//7.07
      const float Q_a_y_KF = 0.0707;//7.07

      const float R_x_KF = 0.5;//5
      const float R_y_KF =  0.5;//5
      const float Q[]=
      { pow(Q_x_KF,2),0,0,0,0,0,
          0,pow(Q_y_KF,2),0,0,0,0,
          0,0,pow(Q_v_x_KF,2),0,0,0,
          0,0,0,pow(Q_v_y_KF,2),0,0,
          0,0,0,0,pow(Q_a_x_KF,2),0,
          0,0,0,0,0,pow(Q_a_y_KF,2)
      };
      memcpy(moving_object_->kalman_pt4_ab[i]->process_noise_cov->data.fl,Q, sizeof(Q));

      const float R[]=
      { pow(R_x_KF,2),0,
          0,pow(R_y_KF,2)
      };
      memcpy(moving_object_->kalman_pt4_ab[i]->measurement_noise_cov->data.fl, R, sizeof(R));

      const float P[]=
      { pow(P_x_KF,2),0,0,0,0,0,
          0,pow(P_y_KF,2),0,0,0,0,
          0,0,pow(P_v_x_KF,2),0,0,0,
          0,0,0,pow(P_v_y_KF,2),0,0,
          0,0,0,0,pow(P_a_x_KF,2),0,
          0,0,0,0,0,pow(P_a_y_KF,2)
      };
      memcpy( moving_object_->kalman_pt4_ab[i]->error_cov_post->data.fl, P, sizeof(P) );

      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[0] = position_measure.x;
      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[1] = position_measure.y;
      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[2] = 0;
      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[3] = 0;
      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[4] = 0;
      moving_object_->kalman_pt4_ab[i]->state_post->data.fl[5] = 0;


      //1
      cvKalmanPredict(moving_object_->kalman_pt4_ab[i], 0);

      moving_object_->motion_state_ab.x_pred_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[0];
      moving_object_->motion_state_ab.y_pred_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[1];

      //2
      CvMat* measurement_matrix = cvCreateMat(2, 1, CV_32FC1);//²âÁ¿ŸØÕó2Î¬£¬x¡¢y×ø±ê
      measurement_matrix->data.fl[0] = position_measure.x;
      measurement_matrix->data.fl[1] = position_measure.y;
      cvKalmanCorrect( moving_object_->kalman_pt4_ab[i], measurement_matrix );

      //3
      cvKalmanCorrect( moving_object_->kalman_pt4_ab[i], measurement_matrix );
      cvReleaseMat( &measurement_matrix );

      moving_object_->motion_state_ab.x_post_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[0];
      moving_object_->motion_state_ab.y_post_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[1];
      moving_object_->motion_state_ab.v_x_pt4[i] = 0;//moving_object->kalman_motion[i]->state_post->data.fl[2];
      moving_object_->motion_state_ab.v_y_pt4[i] = 0;//moving_object->kalman_motion[i]->state_post->data.fl[3];
      moving_object_->motion_state_ab.acc_x_pt4[i] = 0;//moving_object->kalman_motion[i]->state_post->data.fl[2];
      moving_object_->motion_state_ab.acc_y_pt4[i] = 0;//moving_object->kalman_motion[i]->state_post->data.fl[3];
    }
    else{
      cvKalmanPredict(moving_object_->kalman_pt4_ab[i],0);

      moving_object_->motion_state_ab.x_pred_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[0];
      moving_object_->motion_state_ab.y_pred_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[1];

      CvMat* measurement_matrix = cvCreateMat(2,1,CV_32FC1);//

      if (moving_object_->has_match)
      {
        measurement_matrix->data.fl[0] = position_measure.x;
        measurement_matrix->data.fl[1] = position_measure.y;
      }
      else
      {
        measurement_matrix->data.fl[0] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[0];
        measurement_matrix->data.fl[1] = moving_object_->kalman_pt4_ab[i]->state_pre->data.fl[1];
      }

      //3
      cvKalmanCorrect( moving_object_->kalman_pt4_ab[i], measurement_matrix );
      cvReleaseMat( &measurement_matrix );

      moving_object_->motion_state_ab.x_post_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[0];
      moving_object_->motion_state_ab.y_post_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[1];
      moving_object_->motion_state_ab.v_x_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[2];
      moving_object_->motion_state_ab.v_y_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[3];
      moving_object_->motion_state_ab.acc_x_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[4];
      moving_object_->motion_state_ab.acc_y_pt4[i] = moving_object_->kalman_pt4_ab[i]->state_post->data.fl[5];
    }
  }//end for (int i=0; i<4; i++)

  moving_object_->is_KF_pt4_ab_initialized = true;
}

void Motion_global::MotionState_Filter_ab( MovingObject *moving_object_ )
{
  int object_type = abs(moving_object_->object_type);

  /****************************
   * Kalman滤波更新
   ****************************/
  FilterMethold_KF_Center2_ab( moving_object_);

  if (object_type>2)//目标可能较大,才更新head和四个点
  {
    if (moving_object_->filter_pos_head_ab < 0)
    {
      moving_object_->filter_pos_head_ab= 0;
      FilterMethold_KF_PosHead_ab( moving_object_);
    }
    if (moving_object_->filter_pos_head_ab > 0)
      FilterMethold_KF_PosHead_ab( moving_object_);

    // Update 4 bbox corners
    FilterUpdate_Pt4_ab(moving_object_);
  }
}



