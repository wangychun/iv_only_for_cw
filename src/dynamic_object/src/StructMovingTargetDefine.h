#ifndef SRC_DYNAMIC_OBJECT_SRC_STRUCTMOVINGTARGETDEFINE_H_
#define SRC_DYNAMIC_OBJECT_SRC_STRUCTMOVINGTARGETDEFINE_H_
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <list>
#include <vector>
//#include "transform/rigid_transform.h"
#include <opencv2/opencv.hpp>
//Boost
#include <boost/algorithm/string.hpp>
#include <boost/timer.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/asio.hpp>
//PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "tracker/ukf.h"
using namespace boost;

#define MAX_NUM_TARGET          200
#define HISTORY_NUM             100
#define PREDICT_NUM             20

const float pi = 3.1415926;

using std::vector;

#define UNKNOWN				    0
//#define UNIDENTIFY			    1
#define PASSABLE				2
#define GROUND                  3
#define RIGIDNOPASSABLE			4
#define PEOPLENOPASSABLE		5
#define RADARNOPASSABLE			6

//Unit: m
#define OGMSHOW_RESOLUTION      0.15
#define OGM_RESOLUTION          0.2  //栅格地图分辨率,即每个像素表示实际米制距离0.2m
#define OGM_WIDTH               80
#define OGM_HEIGHT              100
#define OFFSET_Y                50

//ego vehicle
#define EGO_CAR_WIDTH           2
#define EGO_CAR_LENGTH          5
#define EGO_CAR_HIGHT           1.6
#define LIDAR_HEIGHT            1.90

enum class CellProperty//每个占据栅格属性
{
  Unknown,
  Unidentify,
  Passable,
  Ground,
  RigidNoPassable,
  MovingTarget
};

/**
 * @brief OGMProperty class for store the occupied grid map properties and complete the
 *        conversion between the image map;
 *        Notes: any function or members contains 'map' means the corresponding image info and
 *        conform to the image coordinate convention
 */
class OGMProperty
{
public:
  OGMProperty();
  OGMProperty(float ogm_width, float ogm_height, float ogm_y_offset, float resolution);

  cv::Size mapSize() const;
  cv::Point mapAnchor() const;
  int mapOffset() const;//from bottom to up
  int mapCellNum() const;
  float ogmYForwardLim() const {return ogm_height_ - ogm_y_offset_; }
  float ogmYBackwardLim() const {return -ogm_y_offset_; }
  //米制单位
  float ogm_width_;
  float ogm_height_;
  float resolution_;//unit pixel equal to how many(resolution_) meter long
  float ogm_y_offset_;
};

class PolarProperty
{
public:
  PolarProperty() = default;
  PolarProperty(float radial_limit, float radial_resolution, float circular_limit, float circular_resolution);

  int radialCellsNum() const;
  int circularCellsNum() const;
  int polarCellsNum() const;

  //米制单位
  float radial_resolution_;//半径分辨率
  float circular_resolution_;//角度分辨率
  float radial_limit_;////径向范围unit pixel equal to how many(resolution_) meter long
  float circular_limit_;//周向范围
};

struct TargetID
{
  int          ID_number;
  bool         locked;
};

struct Point2D //目标矩形框点坐标
{
  float        x;
  float        y;
  int          polar_index;
  bool         is_occluded;//是否被遮挡,初始为true
  operator CvPoint2D32f() const
	    {
    CvPoint2D32f point; point.x = x; point.y = y;
    return point;
	    }
};

struct Point_d
{
  double        x;
  double        y;
};

struct Point2D_Polar//车体坐标系下点
{
  float        angle;
  float        dis_xy;//径向距离
  int          point_index;
  bool         is_occluded;
  float        occluded_disxy;
};

struct Contour_Rect
{
  Point2D     point4[4];//矩形四个点坐标
};

struct Contour_Rect_d
{
  Point_d     point4[4];
};
struct Contour_Polar
{
  Point2D_Polar polar4[4];
};

struct Rect_region 
{
  float        x_min;
  float        x_max;
  float        y_min;
  float        y_max;
};
struct cubic 
{
  float        x_min;
  float        x_max;
  float        y_min;
  float        y_max;
  float        z_min;
  float        z_max;
};

struct Line
{
  CvPoint2D32f points[2];
};

struct Cluster_Index
{
  int end_index;
  int start_index;
  int index_num;
  vector<int> index_vector;
  int type;
};

typedef struct Line_int//int型(图像中)的射线结构体
{
  int          x1;//起点
  int          y1;
  int          x2;//终点
  int          y2;
  float        line_length;
  float        angle;
  int          type;
  int          left_invalid; //
  int          right_invalid;
}Line_int;

struct Line_float
{
  float        x1;
  float        y1;
  float        x2;
  float        y2;
  float        line_length;
  int          type;
  int          left_invalid;
  int          right_invalid;
  float        angle;
};

struct Line_s
{

  float        a; // sin
  float        b; // -cos
  float        c; //ax + by = c
  float        angle;//直线斜率角
  float        line_length;//
  int          type;
  float        d; //y = dx2 + ax + c
};

struct OGM_Cell  //栅格地图每一个cell的属性
{
  CellProperty type;
  float        intensity;
  float        max_z;
  float        min_z;
  float        average_z;//一个栅格中的点云平均高度
  float        delta_z;//一个栅格中点云的最大高度差

  int          layers_num;
  int          points_num;
  int          startlaser_index;
  int          endlaser_index;
  int          endpoint_index;
  int          startpoint_index;

  float        ground_z;

  vector<int>  cloud_index;//存储点云索引,用于保存投影前三维点云信息
};

struct Polar_Cell //极坐标栅格地图cell属性
{
  CellProperty type;
  float        intensity;
  float        max_z;
  float        min_z;
  float        average_z;
  float        delta_z;
  int          points_num;
  bool         up_link;
  bool         down_link;
  bool         left_link;
  bool         right_link;

  float        ground_z;
  float        ground_xy;
  float        grad_m;
  float        grad_b;
};


struct Region_Cell
{
  int          object_num;
  int          object_index_end;
  int          object_index_start;

  int          layers_num;
  int          startlaser_index;
  int          endlaser_index;
  int          endpoint_index;
  int          startpoint_index;

  float        ground_endz;
  float        ground_startz;
  float        disxy_end;
  float        disxy_start;
  int          ground_type;                    //0 unknow, 1 pure ground, 2 with object, 3 pure obstacle

};


struct Pose_IMU
{
  double       dLat;//Y 纬度
  double       dLng;//X 经度
  double       dAld;
  double       heading;             //
  double       pitch;               //
  double       roll;                //
};

struct State_Vehicle //本车车辆状态
{
  double       time_stamp;
  bool         is_update;
//  double       fFLRWheelAverAngle;
  double       fForwardVel;//车速
  Pose_IMU     global_position;//车辆位姿,坐标系原点在后轴中心点
};

namespace sensors_fusion {

/*!
 * @brief Ego vehicle state struct
 */
struct EgoVehicleState
{
  EgoVehicleState();
  EgoVehicleState(const State_Vehicle& st);

  static EgoVehicleState EgoVehicle(const cv::Scalar& vehicleShape);

  //! conversion to the old-style State_Vehicle
  operator State_Vehicle() const;

  // shape info
  cv::Scalar vehicle_shape_; // length, width, height
  // vehicle real time status
  float speed_;
  // vehicle localization
  double longitude_, latitude_, altitude_; //经度, 纬度
  float heading_, pitch_, roll_;
};


} // namespace sensors_fusion



struct Object_Shape//目标形状
{
  Point2D       point4[4];//矩形框四个顶点,顺时针排序
  Point2D_Polar polar4[4];//极坐标,0——最近点 1——角度最小点 2——角度最大点 3——最远点
  Line_s        fit_line4[4];//四条边
  int           polar_state;//确定自身遮挡状态,2-直接观察到一条边,两个顶点,3——两条边,三个顶点

  float         object_width;
  float         object_length;
  float         object_height;                    //物体点云z值最大
  bool          is_entire_high;                 //

  float         line_length0;                    //矩形框边长0-1的边长
  float         line_length1;                     //矩形框边长1-2的边长
  bool          is_entire_line0;                //轮廓是否被扫描完整
  bool          is_entire_line1;

  int           head_index;                     //车头方向,目标运动方向,初始为-1,0-3,四个方向
  bool          is_entire_head;                 //是否已经完全确认车头方向

  cubic         cubic_model_3d;                 //

  int           object_type;                   //目标类别,根据长宽定1-9等级,在TrackingUpdate::ObjectTypeClassify函数中确定
  bool          is_entire_type;                 //
};

enum TrackState
{
  Uninitialized,
  Initializing,
  Tracking,
  Mature,
  Drifting,
  Invalid
};
struct Track_State   //跟踪状态
{       
  int          tracked_times;                  //总跟踪次数，一直累加
  int          tracked_times_c;                //目标连续跟踪次数，目标丢失则置0
  int          missed_times;                   //总丢失次数
  int          missed_times_c;                 //连续丢失次数
  int          tracked_state;                  //跟踪属性0-5个等级,默认为0
  int          confidence_level;               //危险程度
};

struct Predict_traj   //预测轨迹，根据当前状态，预测未来20帧轨迹
{
  double       time_stamp;//每个预测点时间戳，在当前时间戳基础上递增
  Point_d      point; //预测点坐标

  float        v_x;
  float        v_y;
  float        acc_x;
  float        acc_y;
  float        pos_head;
  float        v_w;

  int          confidence_level;
};

struct PredictInfo{
  double time_stamp;//每个预测点时间戳，在当前时间戳基础上递增
  Point_d point; //预测点全局坐标
  float   v_x;
  float   v_y;
};

struct Motion_filter
{
  int                  filter_pos_head;                   //
  int                  filter_state_pt4[4];
  int                  filter_center;

  CvKalman*            kalman_pos_head;                   // pose heading angle
  CvKalman*            kalman_center;
  CvKalman*            kalman_pt4[4];                     // four point and center point
};
struct Motion_State
{
  float        rough_w_position;
  float        rough_v_disxy;                  //rough，粗略状态，两帧相减

  float        rough_v_x;
  float        rough_v_y;
  float        rough_w_head;                   //

  float        x_post;
  float        y_post;
  float        pose_head_post;                 //post，滤波状态

  float        v_x_post;
  float        v_y_post;                        //
  float        acc_x_post;
  float        acc_y_post;                      //
  float        v_w_post;                        //
  float        v_acc;


  float        v_post;
  float        v_theta_post;                    //

  float        x_pred;
  float        y_pred;
  float        x_v_pred;
  float        y_v_pred;
  float        pose_head_pred;
  float        v_w_pred;                        //pred，预测状态

  float        v_pred;
  float        v_theta_pred;

  float        x_post_pt4[4];
  float        y_post_pt4[4];
  float        v_x_pt4[4];
  float        v_y_pt4[4];
  float        acc_x_pt4[4];
  float        acc_y_pt4[4];

  float        x_pred_pt4[4];
  float        y_pred_pt4[4];

};

struct History_State
{
  int           is_show_trajectory;//是否显示历史轨迹
  int           history_num;      //历史跟踪次数

  double        history_time[HISTORY_NUM];       //真实历史更新时间

  float         history_v[HISTORY_NUM];// real vel
  float         history_head[HISTORY_NUM];//0,1
  Contour_Rect  history_rect[HISTORY_NUM];//
  CvPoint2D32f  history_center[HISTORY_NUM];

  Point_d       history_center_ab[HISTORY_NUM];//目标中心点历史全局坐标,米制单位
  Contour_Rect_d history_rect_ab[HISTORY_NUM];

  State_Vehicle history_ego_state[HISTORY_NUM];// ego state
};

struct HistoryInfo{
  double          time_stamp;       //目标时间戳
  float           v_post;
  Point_d         history_center_ab;//目标中心点历史全局坐标,米制单位
  Contour_Rect_d  history_rect_ab; //物体框
};

struct MatchRange_grid
{
  CvPoint2D32f  range_rect4[4];//矩形范围四个点坐标,局部坐标系
  float         range_dis[4];//矩形四条边到中心点的距离
  float         d_radius;
};

struct CandidateObject //检测到的候选目标
{
  double               time_stamp;
  int                  object_type;                    //
  CvRect               object_rect;                    //无向矩形框
  CvBox2D              object_box;                     //有向矩形框
  Object_Shape         shape;

  CvPoint2D32f         center_point;
  CvPoint2D32f         track_point;

  int                  track_index0;
  int                  track_index1;

  int                  grid_num;                       //目标占据栅格点的数量
  int                  max_intensity;                  //回波强度0-255

  float                dis_veh_xy;                     //目标中心点离本车距离
  float                position_angle;                 //中心点方位角
  int                  region_index;                   //暂时没用

  int                  occluded_state;                 //0-8,暂时没用

  int                  match_state;                    //初始为-1,如果匹配上,置为1
  int                  match_index;                    //初始为-1,如果匹配上,置为跟踪目标向量的序号
  int                  point_index_pre2cd;
  MatchRange_grid      match_range;

  /********************
   * 匹配步骤中初始化信息
   ********************/
  Point_d              center_point_ab;                //目标中心点全局坐标
  Contour_Rect_d       contour_rect_ab;               //目标矩形框顶点全局坐标
};

struct DetectionObject//图像中检测到的目标
{
  int                  object_type;                    //
  CvRect               object_rect;                    //无向矩形框
  CvBox2D              object_box;                     //有向矩形框
  Object_Shape         shape;

  CvPoint2D32f         center_point;                  //
  CvPoint2D32f         track_point;                   //

  int                  track_index0;
  int                  track_index1;

  float                fill_rate;                      //点云填充密度
  int                  grid_num;                       //目标占据栅格点的数量
  int                  max_intensity;                  //回波强度0-255

  float                dis_veh_xy;                     //目标中心点离本车距离
  float                position_angle;                 //中心点方位角
  int                  region_index;                   //暂时没用

  int                  occluded_state;                 //0-8,暂时没用

  int                  match_state;                    //初始为-1,如果匹配上,置为1
  int                  match_index;                    //初始为-1,如果匹配上,置为跟踪目标向量的序号
  int                  point_index_pre2cd;
  MatchRange_grid      match_range;

  /********************
   * 匹配步骤中初始化信息
   ********************/
  Point_d              center_point_ab;                //目标中心点全局坐标
  Contour_Rect_d       contour_rect_ab;               //目标矩形框顶点全局坐标
};


struct MovingObject
{
  //zhanghm define
  /**********
   * 目标状态
   *********/
  double              timestamp; //目标当前时间戳
  int                 target_ID; //0-200循环使用
  bool                is_new_add; //是否是新添加目标,还没有跟踪一次,初始状态
  bool                has_match;  //是否有匹配目标(已经不是初始状态了)

  /***************
   * 目标检测信息(观测量）
   **************/
  Point_d              center_pt_meas_ab; //目标中心点测量量px,py,利用匹配状态更新,has_match为真,则该值利用检测信息更新,否则利用kalman估计
  CvPoint2D32f         center_pt;         //当前车体坐标系下米制单位目标中心点


  int                  object_type;                    //
  CvRect               object_rect;                    //没有方向的矩形框
  CvBox2D              object_box;                     //有方向矩形 0 1 width ; 0 3 length
  float                pose_head;                     //目标的航向角
  float                position_angle;                 //目标方位角
  float                dis_veh_xy;                     //目标到车辆坐标系距离
  Object_Shape         shape;                  //

  Contour_Rect_d       contour_rect_ab;
  /**************
   * 目标跟踪器
   **************/
  //Kalman C++写法
  Motion_State         motion_state_ab;               //Kalman运动估计
  cv::KalmanFilter     KF_center_ab;                  //中心点Kalman滤波
  bool                 is_KF_center_ab_initialized;      //是否初始化
  CvKalman*            kalman_pt4_ab[4];              // four point and center point
  bool                 is_KF_pt4_ab_initialized;      //矩形框Kalman滤波器是否初始化
  /****************
   * 目标跟踪状态
   **************/
  // trcaked state informtion
  double               time_stamp;                     //目标当前时间戳
  double               time_stamp_start;                    //第一次检测到的时间戳
  int                  ID_number;                      //目标ID号，0-200循环使用
  Track_State          track_state;                    //

  Motion_State         motion_state;                   //

  std::deque<HistoryInfo>   history_info; //目标历史信息,最多HISTORY_NUM帧,由最历史到最新
  std::vector<PredictInfo>  predict_info; //目标预测信息,预测PREDICT_NUM帧

  Contour_Rect         contour_rect_pre;              //上一时刻目标在当前车体坐标系下坐标
  CvPoint2D32f         center_point_pre;              //

  History_State        history_state;     //历史信息

  int                  track_index0;                  //
  int                  track_index1;

  float                dis_move;                      //两秒内目标的移动距离,1.5米认为不动
  Point_d              center_start_ab;               //第一次检测到的中心点,全局位置，作为参考位置
  Contour_Rect_d       rect_start_ab;                 //
  State_Vehicle        ego_veh_start;                 //第一次检测到动态目标时,本车的全局位置

  int                  point_index_pre2cd;            //之前的到当前检测的目标矩形框顶点标号差值
  //	CvPoint2D32f         delta_point;                    //
  bool                 is_same_diag_index;            //

  Point_d              center_point_ab;               //当前全局位置,利用匹配状态更新,has_match为真,则该值更新
  CvPoint2D32f         center_point;                  //米制单位目标中心点
  CvPoint2D32f         track_point;                   //最近匹配点

  /****************
   * 目标匹配属性
   **************/
  bool                 is_updated;                     //当前运动目标是否有匹配或者是不是新加入目标
  int                  candidate_index;               //匹配上检测目标的索引
  int                  radar_index;
  MatchRange_grid      match_range;                   //最近邻寻找范围
  //    float                similar_value_pre;             //
  /****************
   * 目标运动更新属性
   **************/
  //Kalman滤波——全局更新
  CvKalman*            kalman_pos_head_ab;                // pose heading angle
  //    CvKalman*            kalman_center_ab;                  //中心点Kalman滤波
  int                  filter_pos_head_ab;                //-1,没有启动Kalman滤波，0-启动初始化Kalman滤波,1—Kalman滤波自更新
  //    int                  filter_center_ab;                  //
  int                  filter_state_pt4_ab[4];            //

  //  //Kalman滤波——局部更新
  //  CvKalman*            kalman_pos_head;                   // pose heading angle
  //  CvKalman*            kalman_center;
  //  CvKalman*            kalman_pt4[4];                     // four point and center point
  //  int                  filter_pos_head;               //更新航向角,-1,没有启动Kalman滤波，0-启动初始化Kalman滤波,1—Kalman滤波自更新
  //  int                  filter_state_pt4[4];           //更新4个点
  //  int                  filter_center;

  //    float                post_length2[2];               //
  //    float                v_length2[2];                  //长度变化率
  //    int                  filter_length2[2];             //
  //    CvKalman*            kalman_length2[2];             //对轮廓长度的状态的更新（暂时没用）

  UKF                  ukf_center; //对中心点状态的Kalman滤波

  /****************
   * 目标运动状态判别
   **************/
  int                  motion_behavior;                //TODO:初始好像实在MatchField中置为0？-1——初始状态,判定还未运动 0——静止 1——运动
  int                  dangerous_level;                //

  int                  occluded_state;                 //
  int                  send_times;                     //发送次数, >1,只发送上一帧,第一次发送,发送所有历史帧

  int                  predict_num;                   //预测轨迹点的数量,默认设置为20个点
  //	Line_s               predic_fit_line[2];
  //    Predict_traj         predict_traj[PREDICT_NUM];        // 预测轨迹
  //	Predict_traj         predict_traj_fit[PREDICT_NUM];    // 预测轨迹拟合
  int                  region_index;                   //
};

struct History_traj
{
  Point_d             history_center;
  Contour_Rect_d      history_rect;
  double              history_time;
};

struct Target_output2//发送给规划的信息
{
  int                 line_num;           //发送4个点
  CvPoint2D32f        line_point[4];//发送四个点,大地坐标系一致,相对目标自己中心点坐标
  Point_d             center_point;//全局大地坐标系中心点位置
  int                 object_high;
  int                 object_type;
  int                 ID_number; //

  bool                is_updated;
  int                 tracked_times;
  int                 dangerous_level;
  int                 predict_num;
  Predict_traj        predict_traj[PREDICT_NUM];
  int                 history_num;
  History_traj        history_traj[PREDICT_NUM];

  std::vector<PredictInfo> predict_info;
  std::deque<HistoryInfo>   history_info;//包含了当前帧信息
};

struct MovingTargetSend
{
  float               time_stamp;           //
  int                 target_num;          //
  vector<Target_output2>      target_output;          //
};

struct TargetOutput
{
  int target_ID;
  Point_d              center_pt_ab;      //目标中心点测量量px,py,利用匹配状态更新,has_match为真,则该值利用检测信息更新,否则利用kalman估计
  CvPoint2D32f         center_pt;         //当前车体坐标系下米制单位目标中心点
  CvPoint2D32f       line_point[4];      //发送四个点,大地坐标系一致,相对目标自己中心点坐标
  std::deque<HistoryInfo>   history_info;//包含了当前帧信息
  std::vector<PredictInfo> predict_traj;
};

typedef struct MovingTargetOutput_
{
  int target_num;
  std::vector<TargetOutput> target_output;
}MovingTargetOutput;

struct Similar_Weight
{
  float   similar_weight_x;
  float   similar_weight_y;

  float   similar_weight_poseheading;
  float   similar_weight_v;

  float   similar_weight_width;
  float   similar_weight_length;
  float   similar_weight_high;

  float   similar_weight_color;
};



struct Region_Target
{
  int     candidate_num;
  vector<int> candidate_index_vector;
  int     moving_object_num;
  vector<int> moving_object_index_vector;
};

struct Match_Satate
{
  bool    is_match;//初始状态false
  int     match_index;//初始状态为-1,表示没有匹配目标
  int     similar_num;//匹配区域内有多少个目标
  vector<int> similar_index_vector;//匹配区域内目标的序号
  int     max_similar_num;//最终选择的比较相似的匹配目标
  vector<int> max_similar_index_vector;//精筛选的匹配目标
};

struct MatchState
{
  bool    is_match;//初始状态false
  int     match_index;//初始状态为-1,表示没有匹配目标
  int     similar_num;//匹配区域内有多少个目标
  vector<int> similar_index_vector;//匹配区域内目标的序号
  int     max_similar_num;//最终选择的比较相似的匹配目标
  vector<int> max_similar_index_vector;//精筛选的匹配目标
};

struct MatchStatus
{
  bool is_match;
  int match_index;
};


namespace BaseFunction{

float Angle_atan(float d_x_, float d_y_); //径向偏角，-90-90
float Angle_atan2(float d_x_, float d_y_);//径向偏角，0-360
float Angle_line2(CvPoint2D32f line_1_, CvPoint2D32f line_2_);//两点连线角度
/*!
 * @brief 对循环数据,进行循环计算处理,例如圆周角度,360+1度处理为1度,而不至于超限
 * @param value_ 需要处理的数据
 * @param threshod_pre_ 数据限制范围下限
 * @param threshod_up_  数据限制范围上限
 * @return
 */
int value_in_threshod_int(int value_, int threshod_pre_, int threshod_up_);

float Angle_FitLinePoints(vector<CvPoint2D32f> seed_point_vector_, int point_num_);

/*!
 * @brief ////判断点是否在矩形内
 *原理：利用点在四边形内，则该点与边组成的四个三角形面积（利用叉积）之和等于四边形的面积来判断
      // ABCD点按顺序；对于点在线上的情况，会判断成点在四边形内
 * @param x
 * @param y
 * @param A
 * @param B
 * @param C
 * @param D
 * @return
 */
bool isPointInRect(int x, int y, cv::Point A, cv::Point B, cv::Point C, cv::Point D);

/*!
 * @brief 判断点是否在多边形内(默认四边形）
 * @param x
 * @param y
 * @param vertex
 * @param nvert
 * @return
 * @reference  https://www.codeproject.com/Tips/84226/Is-a-Point-inside-a-Polygon
 */
bool isPointInPolygon(int x, int y, cv::Point vertex[], int nvert = 4);
bool isPointInPolygon(int x, int y, std::vector<cv::Point> vertex);
CvPoint2D32f Point_line2(Line_s line1_, Line_s line2_);
/*!
 * @brief 车体坐标转全局 给定车体坐标系下点坐标和车辆航向及全局位置,计算相对于全局坐标系的点的坐标
 * @param x_
 * @param y_
 * @param ego_state_
 * @param lng_x_
 * @param lat_y_
 */
void point_local_to_global(float x_, float y_, State_Vehicle ego_state_, double* lng_x_, double* lat_y_);
void point_global_to_local(double lng_x_, double lat_y_ ,State_Vehicle ego_state_, float* x_, float* y_);//全局坐标转车体

void point_veh_to_img(IplImage* img_, CvPoint* point_, CvPoint2D32f point_f_, float resolution_, int offest_y_);//车体坐标转栅格
void point_img_to_veh(IplImage* img_, CvPoint point_, CvPoint2D32f* point_f_, float resolution_, int offest_y_);//栅格坐标转车体
/*!
 * @brief 地心地固坐标系转统一横轴墨卡托坐标系
 * @param latitude
 * @param longitude
 * @param e0
 * @param n0
 * @param e
 * @param n
 */
void Position_Trans_From_ECEF_To_UTM(double latitude,double longitude,double e0, double n0, double *e, double *n);//GPS经纬度转米制坐标
//初始化栅格地图每个栅格的属性
void InitOGM( int ogm_cell_size_, OGM_Cell* rigid_ogm_ );
void InitOGM(std::vector<OGM_Cell>& rigid_ogm_);
void InitRegion( int region_cell_size_, Region_Cell* region_ );
//初始化栅格地图每个栅格的属性
void InitPolarOGM(vector<Polar_Cell>& polar_ogm);
void InitPolarOGM( int polarogm_cell_size_, Polar_Cell* polar_ogm_ );
//初始化射线属性
void InitVirtualLine(std::vector<Line_int>& line_int);
void InitVirtualLine(Line_int* line_int,int line_num);

void MovingTarget2Send(MovingObject target_send_, Target_output2* target_output_);
void MovingTarget2Send(const MovingObject& moving_target, TargetOutput& target_send);

/*!
 * @brief 在二值图像中从指定点在指定范围内搜索栅格占据情况
 * @param img_binary
 * @param base_point
 * @param distance
 * @return
 */
std::vector<Line_int> SearchOccupiedLine(const cv::Mat& img_binary,const cv::Point& base_point, float distance);
}//end namespace

#endif
