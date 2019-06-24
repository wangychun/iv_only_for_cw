#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <boost/math/special_functions/round.hpp>

#include "ogm_detector.h"
#include <visualization/visualizer_utils.h>
using namespace cv;
using std::vector; using std::ostringstream; using std::cout; using std::endl;

OGMDetector::OGMDetector(const OGMProperty& ogm_property, const PolarProperty& polar_property, bool is_show):
frame_counter_(0),
cloud_clustered_rgb_(nullptr),
clusterd_cloud_vec_(nullptr),
map_resolution(ogm_property.resolution_),
map_offset_y(ogm_property.mapOffset()),
//polar ogm
polarogm_cell_size(polar_property.polarCellsNum()),
virtual_line_num(polar_property.circularCellsNum()),
virtual_line_resolution(polar_property.circular_resolution_),
is_show_img(is_show)
{

}

OGMDetector::~OGMDetector()
{

}

void OGMDetector::SetInput(int frame_counter_, const Mat& img_ogm, const std::vector<OGM_Cell>& rigid_ogm, Polar_Cell* polar_ogm, const CloudPtr cloudIn)
{
  this->frame_counter_ = frame_counter_;
  this->img_ogm_input_ = img_ogm.clone();
  this->rigid_ogm_vec_ = rigid_ogm;
  this->polar_ogm_ = new Polar_Cell[polarogm_cell_size];
  memcpy(this->polar_ogm_, polar_ogm, polarogm_cell_size*sizeof(Polar_Cell));
  this->cloud_input_ = cloudIn;
  //initialize some variables
  static bool initial = true;
  if(initial) {
    int map_width = this->img_ogm_input_.cols; //401
    int map_height = this->img_ogm_input_.rows;//501
    ego_veh_position = cvPoint(map_width/2, map_height -1 - map_offset_y);//本车在图像中的位置
    ROS_WARN_STREAM("initial size: "<<map_width<<" "<<map_height);
    initial = false;
  }
}

void OGMDetector::SetInput(int frame_counter_, const Mat& img_ogm, const std::vector<OGM_Cell>& rigid_ogm, const vector<Polar_Cell>& polar_ogm, const CloudPtr cloudIn)
{
  this->frame_counter_ = frame_counter_;
  this->img_ogm_input_ = img_ogm.clone();
  this->rigid_ogm_vec_ = rigid_ogm;
  this->polar_ogm_ = new Polar_Cell[polarogm_cell_size];
  for(int i = 0; i < polar_ogm.size(); ++i) {
    polar_ogm_[i].intensity = polar_ogm[i].intensity;
    polar_ogm_[i].type = polar_ogm[i].type;
    polar_ogm_[i].min_z = polar_ogm[i].min_z;
    polar_ogm_[i].max_z = polar_ogm[i].max_z;
    polar_ogm_[i].average_z = polar_ogm[i].average_z;
    polar_ogm_[i].delta_z = polar_ogm[i].delta_z;
    polar_ogm_[i].points_num = polar_ogm[i].points_num;
    polar_ogm_[i].ground_z = polar_ogm[i].ground_z;
    polar_ogm_[i].grad_m = polar_ogm[i].grad_m;
    polar_ogm_[i].grad_b = polar_ogm[i].grad_b;
    polar_ogm_[i].up_link = polar_ogm[i].up_link;
    polar_ogm_[i].down_link =  polar_ogm[i].down_link ;
    polar_ogm_[i].left_link = polar_ogm[i].left_link;
    polar_ogm_[i].right_link = polar_ogm[i].right_link;
  }
  this->cloud_input_ = cloudIn;
  //initialize some variables
  static bool initial = true;
  if(initial) {
    int map_width = this->img_ogm_input_.cols; //401
    int map_height = this->img_ogm_input_.rows;//501
    ego_veh_position = cvPoint(map_width/2, map_height -1 - map_offset_y);//本车在图像中的位置
    ROS_WARN_STREAM("initial size: "<<map_width<<" "<<map_height);
    initial = false;
  }
}

void  OGMDetector::InitLine(Line_s* line_, float x1_, float y1_, float x2_, float y2_)
{
  float temp_x = x2_ - x1_;
  float temp_y = y2_ - y1_;
  float temp_dis = sqrt(temp_x*temp_x + temp_y*temp_y);
  if (temp_dis > 0.1)
  {
    line_->line_length = temp_dis;
    line_->angle = atan2(temp_y, temp_x);
    line_->a = sin(line_->angle);
    line_->b = -cos(line_->angle);
    line_->c = x1_*line_->a + y1_*line_->b;//ax + by = c;xsin - ycos = c
  }
}
CvPoint2D32f  OGMDetector::Point_line2(Line_s line1_, Line_s line2_)
{
  CvPoint2D32f temp_point;
  if ( fabs(line1_.a) < fabs(line1_.b) )
  {
    temp_point.x = line1_.c * line1_.a + line2_.c * line2_.a;
    temp_point.y = (line1_.c - temp_point.x * line1_.a) / line1_.b;
  }
  else
  {
    temp_point.y = line1_.c * line1_.b + line2_.c * line2_.b;
    temp_point.x = (line1_.c - temp_point.y * line1_.b) / line1_.a;
  }
  return temp_point;
}

cv::Mat OGMDetector::PreProcessing(const cv::Mat& img_input, bool is_show_result)
{
  //膨胀和闭运算核定义
  Mat dilate_element = getStructuringElement(MORPH_RECT,Size(3,6),Point(1,3));
  Mat close_element = getStructuringElement(MORPH_RECT,Size(3,6),Point(1,3));
  //1)膨胀运算
  Mat img_dilate,img_close;
  cv::dilate(img_input,img_dilate,dilate_element);
  //2)闭运算消除大部分孔洞
  morphologyEx(img_dilate,img_close,MORPH_CLOSE,close_element);

  //可视化
  if(is_show_result){
    namedWindow("mat_img_dilate",CV_WINDOW_NORMAL);
    imshow("mat_img_dilate",img_dilate);
    namedWindow("mat_img_close",CV_WINDOW_NORMAL);
    imshow("mat_img_close",img_close);
  }
  return img_close;
}

void OGMDetector::Virtual_line_draw(Line_int* virtual_line_, int line_num_, float resolution_, const cv::Mat& img_, int method_)
{
  for (int i=0; i<line_num_; i++){//遍历每个方位射线
    //射线初始化
    virtual_line_[i].x1 = ego_veh_position.x;
    virtual_line_[i].y1 = ego_veh_position.y;
    virtual_line_[i].x2 = ego_veh_position.x;
    virtual_line_[i].y2 = ego_veh_position.y;

    virtual_line_[i].angle = i * resolution_;//resolution_ = 1.0 deg
    float temp_angle = virtual_line_[i].angle * pi / 180;
    float max_dis = 60/map_resolution;//径向最远距离

    for (int j = 6; j < max_dis; ++j){//同一方向径向索引
      CvPoint2D32f temp_point_f;
      temp_point_f.x = ego_veh_position.x + j*cos(temp_angle);
      temp_point_f.y = ego_veh_position.y + j*sin(temp_angle);
      CvPoint temp_point = cvPointFrom32f(temp_point_f);

      virtual_line_[i].x2 = temp_point.x;
      virtual_line_[i].y2 = temp_point.y;
      bool is_break = false;
//      if (temp_point.x > 1/map_resolution &&
//                temp_point.x < ((img_->width-1)/map_resolution) &&
//                temp_point.y > 1/map_resolution &&
//                temp_point.y < ((img_->height -1)/map_resolution)){//在图像范围内
      if (temp_point.x >= 0 &&
          temp_point.x <= (img_.cols-1) &&
          temp_point.y >= 0&&
          temp_point.y <= (img_.rows -1)){//在图像范围内
//        uchar* ptr =(uchar*)(img_->imageData + temp_point.y*img_->widthStep);
//        uchar val1 = ptr[temp_point.x * img_->nChannels];
        uchar val1 = img_.at<uchar>(temp_point.y, temp_point.x);
        if (val1 > 0)
        {
          bool is_find = false;
          float temp_dis_s = 2/map_resolution;
          if(fabs(sin(temp_angle))>0.9)
          {
            temp_dis_s = 3/map_resolution;
          }
          for (int k = 0; k < temp_dis_s; ++k)
          {
            temp_point_f.x = ego_veh_position.x + (j+k)*cos(temp_angle);
            temp_point_f.y = ego_veh_position.y + (j+k)*sin(temp_angle);
            temp_point = cvPointFrom32f(temp_point_f);
            virtual_line_[i].x2 = temp_point.x;
            virtual_line_[i].y2 = temp_point.y;
            if (temp_point.x>0 && temp_point.x< (this->img_ogm_origin_.cols-1)
                && temp_point.y>0 && temp_point.y< (this->img_ogm_origin_.rows -1))
            {
              int val2 = this->img_ogm_origin_.at<uchar>(temp_point.y, temp_point.x);
              if(val2 > 0) {
                is_find = true;
                break;
              }
//              uchar* val2 = ((uchar *)(img_ogm->imageData + temp_point.y*img_ogm->widthStep));
//              if (val2[temp_point.x * img_ogm->nChannels] > 0)
//              {
//                is_find = true;
//                break;
//              }
            } else
            {
              break;
            }
          }
          if (!is_find)
          {
            temp_point_f.x = ego_veh_position.x + (j)*cos(temp_angle);
            temp_point_f.y = ego_veh_position.y + (j)*sin(temp_angle);
            if(fabs(sin(temp_angle))>0.9)
            {
              temp_point_f.x = ego_veh_position.x + (j+3)*cos(temp_angle);
              temp_point_f.y = ego_veh_position.y + (j+3)*sin(temp_angle);
            }
            temp_point = cvPointFrom32f(temp_point_f);
            virtual_line_[i].x2 = temp_point.x;
            virtual_line_[i].y2 = temp_point.y;
          }
          is_break = true;
        }//end if (val1 > 0)
      }//end if
      else{
        break;
      }
      if (is_break)
        break;
    }//end for (int j = 6; j < max_dis; ++j)
    float temp_x = virtual_line_[i].x1 - virtual_line_[i].x2;
    float temp_y = virtual_line_[i].y1 - virtual_line_[i].y2;
    virtual_line_[i].line_length = sqrt(temp_x*temp_x + temp_y*temp_y);
    virtual_line_[i].type = 1;
  }
}

std::vector<Line_int> OGMDetector::SearchOccupiedLine(const cv::Mat& img_binary,const cv::Point& base_point, float distance)
{
  const float angle_resolution = 1.0;//1度
  const float distance_resolution = 0.2;//0.2m
  const int numChannel = floor(360.0/angle_resolution);
  std::vector<Line_int> virtual_line(numChannel);

  int search_distance = floor(distance/distance_resolution);
  for (int i=0; i<virtual_line.size(); i++){//遍历每个方位射线
      //射线初始化
    virtual_line[i].x1 = base_point.x;
    virtual_line[i].y1 = base_point.y;
    virtual_line[i].x2 = base_point.x;
    virtual_line[i].y2 = base_point.y;
    virtual_line[i].angle = i * angle_resolution;//resolution_ = 1.0 deg
    virtual_line[i].type = 0;//未找到状态
    float temp_angle = virtual_line[i].angle * pi / 180;

    for(int j = 0; j < search_distance; ++j){//一个方位上径向查找
      CvPoint2D32f temp_point_f;
      temp_point_f.x = base_point.x + j*cos(temp_angle);
      temp_point_f.y = base_point.y + j*sin(temp_angle);
      CvPoint temp_point = cvPointFrom32f(temp_point_f);

      if (temp_point.x >= 0 &&
          temp_point.x <= (img_binary.cols-1) &&
          temp_point.y >= 0&&
          temp_point.y <= (img_binary.rows -1)){//在图像范围内
         if(img_binary.at<uchar>(temp_point.y,temp_point.x) > 0){
           virtual_line[i].type = 1;//找到占据点
           virtual_line[i].x2 = temp_point.x;
           virtual_line[i].y2 = temp_point.y;
         }
      }
      else{
        break;
      }

      if(virtual_line[i].type == 1)//第一次找到即可
        break;
    }
    //该方位射线其他属性
    float temp_x = virtual_line[i].x1 - virtual_line[i].x2;
    float temp_y = virtual_line[i].y1 - virtual_line[i].y2;
    virtual_line[i].line_length = sqrt(temp_x*temp_x + temp_y*temp_y);
  }
  return virtual_line;
}

void OGMDetector::Virtual_line_cluster(Line_int* virtual_line_, int line_num_, vector<Cluster_Index>* line_cluster_vector_)
{
  int line_cluster_num = 0;
  int start_line_index = 90;
  float dis_threshold = 2/map_resolution;
  Cluster_Index cluster_new;
  bool is_new = true;
  bool is_start = true;
  float line_min_dis = 10000;
  for (int i=0; i<line_num_; i++)
  {
    int index = BaseFunction::value_in_threshod_int(start_line_index + i, 0, virtual_line_num-1);
    int index_up = BaseFunction::value_in_threshod_int(index + 1, 0, virtual_line_num-1);

    if (is_new)//是不是新聚类
    {
      cluster_new.type = 0;
      cluster_new.start_index = index;
      cluster_new.index_num = 0;
      cluster_new.index_vector.clear();
      is_new = false;
      line_min_dis = 10000;
    }
    //射线距离差
    float delta_length = virtual_line_[index_up].line_length - virtual_line_[index].line_length;

    cluster_new.end_index = index;
    cluster_new.index_num++;
    cluster_new.index_vector.push_back(index);
    line_min_dis = min(virtual_line_[index].line_length, line_min_dis);
    if (virtual_line_[index].line_length >= 45/map_resolution)//射线长度大于45m认为无效
      cluster_new.type = -1;//无效射线
    if(cluster_new.type > -1)
    {
      if (virtual_line_[index].line_length < 45/map_resolution)
        cluster_new.type = 1;//有效射线
      if(virtual_line_[index].y2 >= (ego_veh_position.y + 10/map_resolution ) )
      {
        if (virtual_line_[index].x2 < (ego_veh_position.x - 10/map_resolution)
            || virtual_line_[index].x2 > (ego_veh_position.x + 10/map_resolution) )
          cluster_new.type = -1;
      }
      if(virtual_line_[index].y2 < (ego_veh_position.y + 10/map_resolution ) )
      {
        if (virtual_line_[index].x2 < (ego_veh_position.x - 15/map_resolution)
            || virtual_line_[index].x2 > (ego_veh_position.x + 15/map_resolution) )
          cluster_new.type = -1;
      }
    }

    if (fabs(delta_length) > dis_threshold)
    {
      is_new = true;
      line_cluster_vector_->push_back(cluster_new);
      line_cluster_num++;
    }
    if (!is_new && i==(line_num_-1))
    {
      line_cluster_vector_->push_back(cluster_new);
      line_cluster_num++;
    }
  }

  if (1)//补齐0度特殊射线,比较360度和0度射线长度,判断是不是同一个类别射线
  {
    Cluster_Index cluster_start = line_cluster_vector_->at(0);
    Cluster_Index cluster_end = line_cluster_vector_->at(line_cluster_num-1);
    float delta_length = virtual_line_[cluster_start.start_index].line_length
        - virtual_line_[cluster_end.end_index].line_length;
    if (fabs(delta_length) < dis_threshold)
    {
      cluster_end.end_index = cluster_start.end_index;
      cluster_end.index_num = cluster_start.index_num + cluster_end.index_num;
      for (int i=0; i<cluster_start.index_vector.size(); i++)
      {
        int index = cluster_start.index_vector.at(i);
        cluster_end.index_vector.push_back(index);
      }
      line_cluster_vector_->at(line_cluster_num-1) = cluster_end;
      line_cluster_vector_->erase(line_cluster_vector_->begin());
      line_cluster_num--;
    }
  }
  if (0)//显示聚类射线
  {
    int color_index = 0;
    for (int i=0; i<line_cluster_vector_->size(); i++)
    {
      Cluster_Index temp_cluster = line_cluster_vector_->at(i);
      if (temp_cluster.type > -1)
      {
        CvScalar line_color = cvScalar(0,255,0);//绿色
        if (color_index == 1)
          line_color = cvScalar(0,0,255);//红色
        if (color_index == 2)
          line_color = cvScalar(255,0,0);//蓝色
        color_index ++;
        if (color_index>2)
          color_index = 0;
        for (int k=0; k<temp_cluster.index_num; k++)
        {
          int index = temp_cluster.index_vector.at(k);
          CvPoint temp_point1 = cvPoint(virtual_line_[index].x1, virtual_line_[index].y1);
          CvPoint temp_point2 = cvPoint(virtual_line_[index].x2, virtual_line_[index].y2);
          cv::line(this->img_result_show_, temp_point1, temp_point2, line_color,1,8,0);
        }
      }
    }
  }
}


void OGMDetector::Virtual_line(const cv::Mat& binary_img)
{
  if(binary_img.type()!=CV_8UC1)
    throw std::logic_error("wrong image type, should be CV_8UC1");

  Virtual_line_draw(virtual_line_d, virtual_line_num, virtual_line_resolution, binary_img, 2);
  memcpy(virtual_line, virtual_line_d, virtual_line_num*sizeof(Line_int));

  if (0)//绘制射线
  {
    for (int i=0; i<virtual_line_num; i++)
    {
      CvPoint temp_point1 = cvPoint(virtual_line_d[i].x1, virtual_line_d[i].y1);
      CvPoint temp_point2 = cvPoint(virtual_line_d[i].x2, virtual_line_d[i].y2);

      {
        cv::line(this->img_result_show_, temp_point1, temp_point2, cvScalar(0,255,255),1,8,0);
      }
    }
    namedWindow("test3",CV_WINDOW_NORMAL);
    imshow("test3",this->img_result_show_);
    waitKey(5);
  }

  if (0)//新修改绘制射线方法
  {
    cv::Mat temp_virtual_img = binary_img.clone();
    cv::Point base_point(ego_veh_position.x,ego_veh_position.y);
//    cv::Point base_point(200,100);
    vector<Line_int> virtual_line_temp = BaseFunction::SearchOccupiedLine(temp_virtual_img,base_point,20);

    cv::cvtColor(temp_virtual_img,temp_virtual_img,CV_GRAY2BGR);

    for(int i = 0;i<virtual_line_temp.size();++i){
      if(virtual_line_temp.at(i).type == 1){
        cv::Point temp_point1(virtual_line_temp[i].x1, virtual_line_temp[i].y1);
        cv::Point temp_point2(virtual_line_temp[i].x2, virtual_line_temp[i].y2);
        cv::line(temp_virtual_img,temp_point1,temp_point2,cv::Scalar(0,255,255),1);
      }
    }
//    namedWindow("temp_virtual_img",CV_WINDOW_NORMAL);
//    imshow("temp_virtual_img",temp_virtual_img);
    //射线聚类
    vector<Cluster_Index> img_virtual_line_temp;
    this->ClusteringVirtualLine(virtual_line_temp,img_virtual_line_temp);
  }


  line_cluster_vector.clear();
  //射线聚类
  Virtual_line_cluster(virtual_line_d, virtual_line_num, &line_cluster_vector);

  if (1)//对聚类射线两侧射线进行距离判断,剔除不感兴趣目标
  {
    for (int i=0; i<line_cluster_vector.size(); i++)
    {
      Cluster_Index temp_cluster = line_cluster_vector.at(i);
      if(temp_cluster.type > 0)//只分析有效射线,值为1的
      {
        int temp_type = 1;
        int index_start = temp_cluster.index_vector.at(0);
        int index_end = temp_cluster.index_vector.at(temp_cluster.index_num-1);
        int index_pre = BaseFunction::value_in_threshod_int(index_start-1, 0, virtual_line_num-1);
        int index_up = BaseFunction::value_in_threshod_int(index_end+1, 0, virtual_line_num-1);
        float delta_length_up = virtual_line_d[index_up].line_length - virtual_line_d[index_end].line_length;
        float delta_length_pre = virtual_line_d[index_pre].line_length - virtual_line_d[index_start].line_length;
        if ( (delta_length_pre>2/map_resolution && delta_length_up<2/map_resolution)
            || (delta_length_pre<2/map_resolution && delta_length_up>2/map_resolution) )
        {
          temp_type = 2;//左边或右边比终止或起始射线长
        }
        if ((delta_length_pre)>2/map_resolution && (delta_length_up)>2/map_resolution)
        {
          temp_type = 3;//两边射线都比终止或起始射线长
        }
        temp_cluster.type = temp_type;
      }
      for (int k = 0; k < temp_cluster.index_vector.size(); ++k)//对该聚类内部所有射线type进行赋值
      {
        int line_index = temp_cluster.index_vector.at(k);
        if (temp_cluster.type>0)
        {
          virtual_line_d[line_index].type = i;
          virtual_line[line_index].type = -temp_cluster.type;
          virtual_line_type[line_index] = temp_cluster.type;
        } else
        {
          virtual_line_d[line_index].type = -1;
          virtual_line[line_index].type = -(virtual_line_num + 1);
        }
      }
      line_cluster_vector.at(i) = temp_cluster;
    }
  }

  if (0)//可视化
  {
    int color_index = 0;
    for (int i=0; i<line_cluster_vector.size(); i++)
    {
      Cluster_Index temp_cluster = line_cluster_vector.at(i);
      if (temp_cluster.type>0)
      {
        CvScalar color_line = cvScalar(175,175,0);
        if (color_index == 1)
          color_line = cvScalar(255,25,125);
        if (color_index == 2)
          color_line = cvScalar(175,0,175);

        color_index++;
        if (color_index == 3)
          color_index=0;

        for (int k = 0; k < temp_cluster.index_vector.size(); ++k)
        {
          int line_index = temp_cluster.index_vector.at(k);
          CvPoint temp_point1 = cvPoint(virtual_line_d[line_index].x1, virtual_line_d[line_index].y1);
          CvPoint temp_point2 = cvPoint(virtual_line_d[line_index].x2, virtual_line_d[line_index].y2);

          if (temp_cluster.type>0)
          {
            cv::line(this->img_result_show_, temp_point1, temp_point2, color_line,1,8,0);
          }
        }
        if (0)
        {
//          CvFont font;
//          cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.8f, 1.0f, 0, 1, 8);
          char file_name_ID[50];
          sprintf(file_name_ID, "%d", temp_cluster.index_num);
          CvScalar word_color = cvScalar( 0, 0, 255 );

          int line_index = temp_cluster.index_vector.at(0);
          CvPoint word_point = cvPoint(virtual_line_d[line_index].x2, virtual_line_d[line_index].y2);

//          cvPutText(this->img_result_show_, file_name_ID, word_point, &font, word_color);
          cv::putText(this->img_result_show_,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,word_color,2);
        }
      }
    }
  }
}

void OGMDetector::ClusteringVirtualLine(const vector<Line_int>& virtual_line_vector,vector<Cluster_Index>& line_cluster_vector)
{
  vector<Cluster_Index>().swap(line_cluster_vector);//清空向量
  float dis_threshold = 2.0/this->map_resolution;//距离阈值
  Cluster_Index cluster_new;
  bool is_new = true;//新目标
  //从0遍历到不包括最后一个射线
  //1)先对整个射线中向量进行聚类
  for(auto iter = virtual_line_vector.begin();iter!=virtual_line_vector.end()-1;++iter){
//    int index = std::distance(iter,virtual_line_vector.begin());//TODO:待验证
    int index = iter - virtual_line_vector.begin();
    if(iter->type == 0)
      continue;
    if(is_new){
      cluster_new.start_index = cluster_new.end_index = index;
      cluster_new.index_num = 1;
      cluster_new.index_vector.clear();
      cluster_new.index_vector.push_back(index);
      is_new = false;
    }
    auto iter_next = iter + 1;//从当前位置往后遍历,比较相邻两射线
    float delta_distance = fabs(iter->line_length - iter_next->line_length);

    if((delta_distance <= dis_threshold)&&(iter_next->type==1)){//属于同一目标
      cluster_new.index_num++;
      cluster_new.end_index = index + 1;
      cluster_new.index_vector.push_back(index + 1);
    }
    else{
      line_cluster_vector.push_back(cluster_new);
      is_new = true;
    }

    if(iter_next == virtual_line_vector.end()-1&&is_new){
      if(iter_next->type == 1){//单独成为一个
        cluster_new.start_index = cluster_new.end_index = index;
        cluster_new.index_num = 1;
        cluster_new.index_vector.clear();
        cluster_new.index_vector.push_back(index);
        line_cluster_vector.push_back(cluster_new);
      }
    }
    else if(iter_next == virtual_line_vector.end()-1&&!is_new){
      line_cluster_vector.push_back(cluster_new);
    }
  }

  //2)补齐0度特殊射线,比较360度和0度射线长度,判断是不是同一个类别射线
  int start_index = virtual_line_vector.front().type;
  int end_index = virtual_line_vector.back().type;
  if(start_index==1&&end_index==1){//首尾都有有效射线
    float delta_length = fabs(virtual_line_vector.front().line_length - virtual_line_vector.back().line_length);
    if(delta_length < dis_threshold){//两个目标应该是同一个目标
      //把最后一个目标添加到第一个目标上
      for(const auto& idx:line_cluster_vector.front().index_vector){
        line_cluster_vector.back().index_vector.push_back(idx);
        line_cluster_vector.back().index_num++;
      }
      line_cluster_vector.erase(line_cluster_vector.begin());//删除最后一个目标
    }
  }

  //可视化
  if (0)//显示聚类射线
  {
    cv::Mat img_clustered = img_ogm_origin_.clone();
    cv::cvtColor(img_clustered,img_clustered,CV_GRAY2BGR);
    int color_index = 0;
    for (int i=0; i<line_cluster_vector.size(); i++)
    {
      Cluster_Index temp_cluster = line_cluster_vector.at(i);
      CvScalar line_color = cvScalar(0,255,0);//绿色
      if (color_index == 1)
        line_color = cvScalar(0,0,255);//红色
      if (color_index == 2)
        line_color = cvScalar(255,0,0);//蓝色
      color_index ++;
      if (color_index>2)
        color_index = 0;
      for (int k=0; k<temp_cluster.index_num; k++)
      {
        int index = temp_cluster.index_vector.at(k);
        CvPoint temp_point1 = cvPoint(virtual_line_vector[index].x1, virtual_line_vector[index].y1);
        CvPoint temp_point2 = cvPoint(virtual_line_vector[index].x2, virtual_line_vector[index].y2);
        cv::line(img_clustered, temp_point1, temp_point2, line_color,1,8,0);
      }
    }

    namedWindow("clustered_image",CV_WINDOW_NORMAL);
    imshow("clustered_image",img_clustered);
    waitKey(10);
  }
}

int OGMDetector::ObjectType_Classify(const float& object_with_, const float& object_length_)
{
  float temp_length = max(object_with_, object_length_);//最长边长
  int temp_object_type = 1;

  if (temp_length<=0.7)
    temp_object_type = 1;	// 1

  if (temp_length>0.7 && temp_length<=1.3)
    temp_object_type = 2;   // 2

  if (temp_length>1.3 && temp_length<=2.5)
    temp_object_type = 3;   // 3

  if (temp_length>2.5 && temp_length<=3.9)
    temp_object_type = 4;   // 4

  if (temp_length>3.9 && temp_length<=6.5)
    temp_object_type = 5;   // 5

  if (temp_length>6.5 && temp_length<=8.5)
    temp_object_type = 6;   // 5

  if (temp_length>8.5 && temp_length<=12.5 )
    temp_object_type = 7;   // 7

  if (temp_length>12.5 && temp_length<=18.5 )
    temp_object_type = 8;   //


  if ((temp_length>18.5))
    temp_object_type = 9;   //

  return temp_object_type;
}

void OGMDetector::TransformPolarPoint(CandidateObject* temp_object)
{
  CvPoint2D32f box_point4[4];
  cvBoxPoints(temp_object->object_box, box_point4);//得到矩形包围盒的四个顶点

  temp_object->shape.polar4[0].dis_xy = 1000;
  float min_angle = 361;
  float max_angle = -361;
  temp_object->shape.polar4[3].dis_xy = 0;
  Point2D_Polar temp_contours_polar4[4];
  for (int m=0; m<4; m++)//找出矩形框四个点对应的最大距离,最大最小方位角顶点标号
  {
    CvPoint2D32f temp_point = box_point4[m];
    float temp_x = temp_point.x - ego_veh_position.x;//车体坐标系下点
    float temp_y = ego_veh_position.y - temp_point.y;
    float temp_dis_xy = sqrt(temp_x * temp_x + temp_y * temp_y);
    float temp_angle = BaseFunction::Angle_atan2(temp_x, temp_y);

    float delta_angle_center = temp_angle - temp_object->position_angle;//矩形顶点与中心点的角度差
    if(delta_angle_center > 180)
      delta_angle_center = delta_angle_center - 360;

    if(delta_angle_center < -180)
      delta_angle_center = delta_angle_center + 360;

    temp_contours_polar4[m].angle = temp_angle;
    temp_contours_polar4[m].dis_xy = temp_dis_xy;
    temp_contours_polar4[m].point_index = m;

    if (temp_dis_xy < temp_object->shape.polar4[0].dis_xy)//最小距离点
    {
      temp_object->shape.polar4[0].dis_xy = temp_dis_xy;
      temp_object->shape.polar4[0].angle = temp_angle;
      temp_object->shape.polar4[0].point_index = m;
    }
    if (temp_dis_xy > temp_object->shape.polar4[3].dis_xy)//最大距离点
    {
      temp_object->shape.polar4[3].dis_xy = temp_dis_xy;
      temp_object->shape.polar4[3].angle = temp_angle;
      temp_object->shape.polar4[3].point_index = m;
    }
    if (delta_angle_center < min_angle)//min_angle
    {
      temp_object->shape.polar4[1].dis_xy = temp_dis_xy;
      temp_object->shape.polar4[1].angle = temp_angle;
      temp_object->shape.polar4[1].point_index = m;
      min_angle = delta_angle_center;
    }
    if (delta_angle_center > max_angle)//max_angle
    {
      temp_object->shape.polar4[2].dis_xy = temp_dis_xy;
      temp_object->shape.polar4[2].angle = temp_angle;
      temp_object->shape.polar4[2].point_index = m;
      max_angle = delta_angle_center;
    }

  }//end for (int m=0; m<4; m++)

  //跨0度时验证
  if (temp_object->shape.polar4[2].angle - temp_object->shape.polar4[1].angle > 180)
  {
    Point2D_Polar temp_polar1 = temp_object->shape.polar4[1];
    temp_object->shape.polar4[1] = temp_object->shape.polar4[2];
    temp_object->shape.polar4[2] = temp_polar1;
  }

  temp_object->shape.polar_state = 3;
  if (temp_object->shape.polar4[0].point_index == temp_object->shape.polar4[1].point_index
      || temp_object->shape.polar4[0].point_index == temp_object->shape.polar4[2].point_index)
    temp_object->shape.polar_state = 2;
}

void OGMDetector::Identify_PoseHead( CandidateObject* temp_object)
{
  if (temp_object->shape.object_type>2)
  {
    float delata_length = temp_object->shape.line_length0 - temp_object->shape.line_length1;
    if(temp_object->shape.is_entire_line0 && temp_object->shape.is_entire_line1)
    {
      if(delata_length > 1)
        temp_object->shape.head_index = 1;

      if(delata_length < -1)
        temp_object->shape.head_index = 0;
    }
    if (temp_object->shape.line_length0 > 4.0 || temp_object->shape.line_length1>4.0)
    {
      if(temp_object->shape.line_length0 >  temp_object->shape.line_length1 && fabs(delata_length) >1)
        temp_object->shape.head_index = 1;

      if(temp_object->shape.line_length1 > temp_object->shape.line_length0 && fabs(delata_length) >1)
        temp_object->shape.head_index = 0;
    }
  }
}

void OGMDetector::InitCandidate( CvRect object_rect_, CvBox2D object_box_,CandidateObject* temp_object)
{
  temp_object->object_rect = object_rect_;
  temp_object->object_box = object_box_;

  temp_object->center_point.x = temp_object->object_box.center.x;
  temp_object->center_point.y = temp_object->object_box.center.y;

  memset(&temp_object->shape, 0 , sizeof(Object_Shape) );
  CvPoint2D32f point4_f[4];
  cvBoxPoints(temp_object->object_box, point4_f);
  for (int m=0; m<4; m++)
  {
    temp_object->shape.point4[m].x = point4_f[m].x;
    temp_object->shape.point4[m].y = point4_f[m].y;
    temp_object->shape.point4[m].is_occluded = true;
    temp_object->shape.polar4[m].is_occluded = true;
  }
  for (int m=0; m<4; m++)
  {
    int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
    InitLine(&temp_object->shape.fit_line4[m], point4_f[m].x, point4_f[m].y, point4_f[m_up].x, point4_f[m_up].y);
    //TODO:角度如何定义
    temp_object->shape.fit_line4[m].angle =  temp_object->object_box.angle*pi/180 + m*pi/2;
    temp_object->shape.fit_line4[m].line_length = sqrt( (point4_f[m].x-point4_f[m_up].x)*(point4_f[m].x-point4_f[m_up].x)
        + (point4_f[m].y-point4_f[m_up].y)*(point4_f[m].y-point4_f[m_up].y) );
  }

  temp_object->shape.line_length0 = temp_object->shape.fit_line4[0].line_length;
  temp_object->shape.line_length1 = temp_object->shape.fit_line4[1].line_length;
  //物体类型判别
  float min_length = min(temp_object->object_box.size.width, temp_object->object_box.size.height);
  float max_length = max(temp_object->object_box.size.width, temp_object->object_box.size.height);
  temp_object->shape.object_type = ObjectType_Classify(temp_object->object_box.size.width*map_resolution,
                                                       temp_object->object_box.size.height*map_resolution);

  temp_object->shape.head_index = -1;
  temp_object->object_type = temp_object->shape.object_type;
  float temp_x = temp_object->center_point.x - ego_veh_position.x;//车体坐标系下x坐标
  float temp_y = ego_veh_position.y - temp_object->center_point.y;//车体坐标系下y坐标
  temp_object->dis_veh_xy = (float)sqrt( temp_x * temp_x + temp_y * temp_y );//距离本车距离
  temp_object->position_angle = BaseFunction::Angle_atan2(temp_x, temp_y);//0-360,顺时针

  //跟踪点确定
  temp_object->track_point = temp_object->center_point;//用的是中心点进行跟踪
  temp_object->occluded_state = 0;

  temp_object->track_index0 = 0;
  temp_object->track_index1 = 0;
  memset(&temp_object->match_range, 0, sizeof(MatchRange_grid) );
}

void OGMDetector::TargetContourClustering(const cv::Mat& img_binary, bool is_show)
{
  dynamic_object_tracking::VisualizerUtils* vis_result_ = dynamic_object_tracking::VisualizerUtils::getInstance();
  //存储轮廓点
  vector<vector<Point>> contours;
  Mat img_contour_show = this->img_color_ogm_.clone();
  vis_result_->DrawEgoVehicle(img_contour_show);

  Mat img_rectangle_show = this->img_color_ogm_.clone();
  vis_result_->DrawEgoVehicle(img_rectangle_show);
  findContours(img_binary, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
//  cv::drawContours(img_contour_show, contours, -1, Scalar(0,255,255));
//  namedWindow("img_contour_show", CV_WINDOW_NORMAL);
//  imshow("img_contour_show", img_contour_show);

  for (auto iter = contours.begin(); iter!= contours.end(); ++iter) {//遍历找到的轮廓
    //寻找包围轮廓矩形框
    cv::Rect object_rect = cv::boundingRect(*iter);//无向矩形
    cv::RotatedRect object_box = cv::minAreaRect(*iter);
//    cv::rectangle(img_contour_show, object_rect, Scalar(0,255,255));
    Point2f vertices[4];
    object_box.points(vertices);
    for (int i = 0; i < 4; i++)
        line(img_rectangle_show, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));

    /*********************
     * 初始化检测目标相关属性
     *********************/
    CandidateObject temp_object;//候选检测目标

    InitCandidate(object_rect, object_box, &temp_object);//
    TransformPolarPoint( &temp_object );//判断矩形顶点角度和距离最小点,给shape.polar4赋值

    /*********************
     * 进行遮挡目标剔除
     *********************/
    //矩形框长和宽
    float min_length = temp_object.shape.object_width;
    float max_length = temp_object.shape.object_length;
    if ((min_length>4.5/map_resolution) || max_length > 20/map_resolution)
    {
      continue;
    }
    bool is_target = false;
    //用射线判断矩形框是否被遮挡
    float temp_dis_xy = temp_object.shape.polar4[0].dis_xy;
    int line_index = (int)(temp_object.position_angle/ virtual_line_resolution);//中心点方位
    int line_index1 = (int)(temp_object.shape.polar4[1].angle/ virtual_line_resolution);//角度最小点
    int line_index2 = (int)(temp_object.shape.polar4[2].angle/ virtual_line_resolution);//角度最大点
    //筛选候选目标(不考虑被遮挡目标)
    if ( (temp_object.center_point.x < (ego_veh_position.x + 30/map_resolution)&&
        temp_object.center_point.x > (ego_veh_position.x - 30/map_resolution))&&//只考虑横向30m内的
        (temp_dis_xy < (virtual_line_d[line_index].line_length + 1/map_resolution)//是不是射线以外目标
            || temp_dis_xy < (virtual_line_d[line_index1].line_length + 1/map_resolution)
            || temp_dis_xy < (virtual_line_d[line_index2].line_length + 1/map_resolution) ) )
    {
      if ( virtual_line_type[line_index]>1)//感兴趣目标
      {
        is_target = true;
        if (temp_object.object_type < 4&&//小目标
            (temp_object.center_point.x > (ego_veh_position.x + 20/map_resolution)||//左右20米以外的小目标不考虑
                temp_object.center_point.x < (ego_veh_position.x - 20/map_resolution)))
        {
          is_target = false;
        }
      }
    }

    /**********************
     * 判断认为是目标矩形框
     **********************/
    if(is_target){
      /**********************
       * 添加到候选目标序列中
       **********************/
      this->total_object_vector.push_back(temp_object);
      //可视化
      Point2f vertices[4];
      cv::RotatedRect object_box(temp_object.object_box);
      object_box.points(vertices);
//      for (int i = 0; i < 4; i++)
//        line(img_contour_show, vertices[i], vertices[(i+1)%4], Scalar(0,255,0));
    }
  }//end for(auto iter = contours.begin(); iter!= contours.end(); ++iter)

  /// ------------ Begin save images for papers writing-----------
  int key = cv::waitKey(5);
  std::stringstream file_name_str;
  if (key == 'd') {
    // 1) Save orgin ogm image
    file_name_str << frame_counter_<<"_origin_ogm_image"<<".png";
    cv::imwrite(file_name_str.str(), this->img_ogm_origin_);
    file_name_str.str("");

    // 2) Save after morphological image
    file_name_str << frame_counter_ << "_morph_image"<<".png";
    cv::imwrite(file_name_str.str(), img_binary);
    file_name_str.str("");

    // 3) Save after contour image
    file_name_str << frame_counter_ << "_contour_image"<<".png";
    cv::imwrite(file_name_str.str(), img_contour_show);
    file_name_str.str("");

    // 4) Save after rectangle image
    file_name_str << frame_counter_ << "_rectangle_image"<<".png";
    cv::imwrite(file_name_str.str(), img_rectangle_show);
    file_name_str.str("");
  }
}

void OGMDetector::TargetIdentify(const std::vector<OGM_Cell>& rigid_ogm_, Polar_Cell* polar_ogm_)
{
  for(int i=0; i<total_object_vector.size(); ++i) {//遍历检测目标
    CandidateObject temp_object = total_object_vector.at(i);
    vector<CvPoint2D32f> grid_point_vector;//存储每个目标所有栅格点
    //得到障碍物栅格点，修正矩形框cv_rect 该步骤通过将temp_object.object_type置为负的,标志要剔除目标
    Grid_Points(&temp_object,&grid_point_vector, rigid_ogm_, polar_ogm_);
    if (temp_object.object_type > 0) {
      int index0 = temp_object.shape.polar4[0].point_index;
      CvPoint2D32f base_point;//距离最近点作为基准点
      base_point.x = temp_object.shape.point4[index0].x;
      base_point.y = temp_object.shape.point4[index0].y;
      float fit_angle= 0;
      Grid_fit_line(& temp_object, grid_point_vector,&fit_angle, &base_point);//拟合矩形边,得到角度,基准点
      CorrectShape(&temp_object, grid_point_vector, fit_angle, base_point);//根据航向角,得到精细矩形轮廓,修正cv_box
      //对修正完之后矩形框重新计算角度最大最小,距离最近点
      TransformPolarPoint(&temp_object);//根据修正结果，重新寻找栅格点
      OcclusionState(&temp_object, polar_ogm_);//矩形角点是否被遮挡
      Trackpoint_index(&temp_object, 2);//找可靠跟踪点
      total_object_vector.at(i) = temp_object;//判断目标属性,目标周围是否被障碍物包围,是为-1
    }
    else if(temp_object.object_type < 0 ) {
      total_object_vector.erase(total_object_vector.begin() + i);
      i--;
    }
  }//end for (int i=0; i<total_object_vector.size(); i++)
}


void OGMDetector::Grid_Points(CandidateObject* temp_object, vector<CvPoint2D32f> *grid_point_vector_,
                                 const std::vector<OGM_Cell>& rigid_ogm_, Polar_Cell* polar_ogm_)
{
  int map_width = this->img_ogm_input_.cols; //401
  int map_height = this->img_ogm_input_.rows;//501
  int rigidogm_cell_size = map_width*map_height;//直角栅格地图栅格总数
  CvPoint2D32f box_point4[4];
  CvPoint2D32f box_center = temp_object->center_point;
  cvBoxPoints(temp_object->object_box, box_point4);

  float cell_max_z = -1000;
  float cell_min_z = 1000;
  float cell_delta_z = 0;
  int grid_num = 0;
  int grid_min_x = temp_object->object_rect.x + temp_object->object_rect.width;
  int grid_max_x = temp_object->object_rect.x;
  int grid_min_y = temp_object->object_rect.y + temp_object->object_rect.height;
  int grid_max_y = temp_object->object_rect.y;


  int rect_min_x = temp_object->object_rect.x;
  int rect_min_y = temp_object->object_rect.y;
  int rect_max_x = temp_object->object_rect.x + temp_object->object_rect.width+1;
  int rect_max_y = temp_object->object_rect.y + temp_object->object_rect.height+1;
  if (rect_max_x > map_width)
    rect_max_x = map_width;

  for (int j = rect_min_y; j< rect_max_y; j++)//遍历目标矩形框区域
  {
    for (int i = rect_min_x; i< rect_max_x; i++)
    {
      int rigid_height = map_height - 1 - j;//像素y值
      int rigid_index = rigid_height * map_width + i;//计算在栅格地图上的位置
      if (rigid_index < rigidogm_cell_size)
      {
        //取出栅格地图像素值
//        uchar* val = ((uchar *)(img_ogm->imageData + j*img_ogm->widthStep)[i*img_ogm->nChannels]);
        int val = this->img_ogm_origin_.at<uchar>(j, i);
        if (val > 0)//是障碍物栅格
        {
          grid_num ++;
          //找到x,y最大最小坐标值
          grid_min_x = min(i, grid_min_x);
          grid_max_x = max(i, grid_max_x);
          grid_min_y = min(j, grid_min_y);
          grid_max_y = max(j, grid_max_y);
          //属于该目标最大最小z值和高度差
          cell_min_z = min(rigid_ogm_[rigid_index].ground_z, cell_min_z);
          cell_max_z = max(rigid_ogm_[rigid_index].max_z,cell_max_z);
          cell_delta_z = max(cell_delta_z, rigid_ogm_[rigid_index].delta_z);

          CvPoint2D32f grid_point = cvPoint2D32f(i, j);
          grid_point_vector_->push_back(grid_point);
        }
      }
    }
  }


  temp_object->grid_num = grid_num;
  float delta_gird_x = grid_max_x - grid_min_x;
  float delta_grid_y = grid_max_y - grid_min_y;
  if (1)
  {
    /***********
     * 目标剔除操作
     ***********/
    float max_length = max(delta_gird_x, delta_grid_y);
    //较大目标,但高度小于1m
    if (temp_object->object_type >6 && temp_object->shape.object_height < 1)
    {
      temp_object->object_type = -1*abs(temp_object->object_type);
    }
    if(grid_num < max_length-3)//栅格点数量不足以构成一条边
    {
      temp_object->object_type = -1*abs(temp_object->object_type);
    }
  }

  temp_object->shape.cubic_model_3d.z_max = cell_max_z;
  temp_object->shape.cubic_model_3d.z_min = cell_min_z;
  temp_object->shape.object_height = cell_max_z;
  if(temp_object->shape.object_height<0)
  {
    temp_object->shape.cubic_model_3d.z_max = 1.5;
    temp_object->shape.cubic_model_3d.z_min = 0;
    temp_object->shape.object_height = 1.5;
  }

  if(temp_object->object_type>0)//根据实际栅格点修正有向矩形框
  {
    temp_object->object_rect.x = grid_min_x-1;
    temp_object->object_rect.y = grid_min_y-1;
    temp_object->object_rect.width = grid_max_x - grid_min_x + 3;
    temp_object->object_rect.height = grid_max_y - grid_min_y + 3;
  }
}


void OGMDetector::Grid_fit_line(CandidateObject* temp_object,
                                   vector<CvPoint2D32f> grid_point_vector_,
                                   float* line_angle_, CvPoint2D32f* base_point_)
{
  int fit_method = 0;
  int point_index1 = temp_object->shape.polar4[1].point_index;
  int point_index2 = temp_object->shape.polar4[2].point_index;
  int point_index0 = temp_object->shape.polar4[0].point_index;

  bool is_fit_line = true;
  if (1)
  {
    int shape_line_index = point_index0;
    if (point_index0== point_index1 || point_index0==point_index2)//只扫描到目标一条边,即最近点会和角度最小或最大点重合
    {
      int delta_index = point_index2 - point_index1;
      shape_line_index = point_index2;
      if (delta_index==1 || delta_index==-3)
        shape_line_index = point_index1;
    }
    else{
      //取较短边
      shape_line_index = point_index0;
      if (temp_object->shape.fit_line4[point_index0].line_length < temp_object->shape.fit_line4[point_index1].line_length)
      {
        int delta_index = point_index2 - point_index0;
        shape_line_index = point_index2;
        if (delta_index==1 || delta_index==-3)
          shape_line_index = point_index1;
      }
    }
    Line_s shape_base_line = temp_object->shape.fit_line4[shape_line_index];//拿到边长
    if (shape_base_line.line_length < 1.9/map_resolution)//边长小于1.9,则认为最小面积拟合基本吻合,不进行重新拟合
      is_fit_line = false;
    int min_rect_width = min(temp_object->object_rect.width, temp_object->object_rect.height);
    int max_rect_height = max(temp_object->object_rect.width, temp_object->object_rect.height);
    if (min_rect_width<0.7/map_resolution && max_rect_height<1.3/map_resolution)//小目标不进行边缘拟合
    {
      is_fit_line = false;
    }
  }

  vector<CvPoint2D32f> seed_point_vector;//目标边缘栅格点
  seed_point_vector.clear();
  float min_dis = 100000;
  float max_dis = -1;
  float min_angle = 361;
  float max_angle = -361;
  CvPoint2D32f start_point, end_point, corner_point, mindis_point;

  if (is_fit_line)//修正最小面积矩形
  {
    Line_int* object_virtual_line = new Line_int[virtual_line_num];
    memset(object_virtual_line, 0, virtual_line_num*sizeof(Line_int));
    int line_start = virtual_line_num-1;
    int line_end = 0;
    //得到对应目标栅格地图下角度最大最小点
    for (int i = 0; i < grid_point_vector_.size(); ++i)
    {
      //取到角度最大，最小,距离最近栅格点
      CvPoint2D32f grid_point = grid_point_vector_.at(i);
      float temp_x = grid_point.x - ego_veh_position.x ;
      float temp_y = ego_veh_position.y - grid_point.y;
      float temp_dis_xy = sqrt(temp_x * temp_x + temp_y * temp_y);
      float temp_angle = BaseFunction::Angle_atan2(temp_x, temp_y);
      min_dis = min(min_dis, temp_dis_xy);
      max_dis = max(max_dis, temp_dis_xy);
      float delta_angle_center = temp_angle - temp_object->position_angle;
      if(delta_angle_center > 180)
        delta_angle_center = delta_angle_center - 360;
      if(delta_angle_center < -180)
        delta_angle_center = delta_angle_center + 360;
      if (delta_angle_center < min_angle)
      {
        min_angle = delta_angle_center;
        start_point = grid_point;
      }
      if (delta_angle_center > max_angle)
      {
        max_angle = delta_angle_center;
        end_point = grid_point;
      }

      int line_index = (int)(temp_angle / virtual_line_resolution);
      line_start = min(line_start, line_index);
      line_end = max(line_end, line_index);
      if (object_virtual_line[line_index].type < 1)//对该射线角度下第一个点进行初始化
      {
        object_virtual_line[line_index].x1 = ego_veh_position.x;
        object_virtual_line[line_index].y1 = ego_veh_position.y;
        object_virtual_line[line_index].x2 = grid_point.x;
        object_virtual_line[line_index].y2 = grid_point.y;
        object_virtual_line[line_index].line_length = temp_dis_xy;
        object_virtual_line[line_index].type = 1;
      }

      if (object_virtual_line[line_index].line_length > temp_dis_xy
          && object_virtual_line[line_index].type >0 )
      {
        object_virtual_line[line_index].x1 = ego_veh_position.x;
        object_virtual_line[line_index].y1 = ego_veh_position.y;
        object_virtual_line[line_index].x2 = grid_point.x;
        object_virtual_line[line_index].y2 = grid_point.y;
        object_virtual_line[line_index].line_length = temp_dis_xy;
        object_virtual_line[line_index].type = 1;
      }
    }

    int line_num =  line_end - line_start;
    if (line_num > virtual_line_num/2)
    {
      int temp_index = line_start;
      line_start = line_end;
      line_end = temp_index;
    }
    line_num = BaseFunction::value_in_threshod_int(line_end - line_start, 0, virtual_line_num-1) + 1;
    if (1)//取出目标边缘栅格点
    {
      Line_s start_base_line = {0};
      InitLine(&start_base_line, start_point.x, start_point.y, end_point.x, end_point.y);
      float max_dis_line = 0;
      float temp_center_c = ego_veh_position.x*start_base_line.a + ego_veh_position.y*start_base_line.b - start_base_line.c;
      for (int i=0; i<grid_point_vector_.size(); i++)
      {
        CvPoint2D32f temp_seed_point = grid_point_vector_.at(i);
        float temp_x = temp_seed_point.x - ego_veh_position.x;
        float temp_y = ego_veh_position.y - temp_seed_point.y;
        float temp_dis_xy = sqrt(temp_x * temp_x + temp_y * temp_y);
        float temp_angle = BaseFunction::Angle_atan2(temp_x, temp_y);
        bool is_seed = true;
        int line_index = (int) (temp_angle / virtual_line_resolution);
        CvPoint temp_point1 = cvPointFrom32f(temp_seed_point);
        float temp_dis_threshod = temp_dis_xy - object_virtual_line[line_index].line_length + 1/map_resolution;
        for (int j = 0; j < temp_dis_threshod; ++j)
        {
          CvPoint2D32f temp_point_f;
          temp_point_f.x = ego_veh_position.x + (temp_dis_xy-j-1)*cos(temp_angle*pi/180);
          temp_point_f.y = ego_veh_position.y + (temp_dis_xy-j-1)*sin(temp_angle*pi/180);
          CvPoint temp_point = cvPointFrom32f(temp_point_f);
//          uchar* val1 = ((uchar *)(img_ogm->imageData + temp_point.y*img_ogm->widthStep)[temp_point.x * img_ogm->nChannels]);
          int val1 = this->img_ogm_origin_.at<uchar>(temp_point.y, temp_point.x);
          if (val1 > 0 && !(temp_point.x== temp_point1.x && temp_point.y==temp_point.y))
          {
            is_seed = false;
            break;
          }
        }
        float temp_dis_line = (temp_seed_point.x*start_base_line.a + temp_seed_point.y*start_base_line.b - start_base_line.c);
        if ( (is_seed && (temp_dis_line*temp_center_c>=0 || fabs(temp_dis_line) <0.2/map_resolution) )
            || (temp_point1.x ==object_virtual_line[line_index].x2 && temp_point1.y == object_virtual_line[line_index].y2))
        {
          if (fabs(temp_dis_line) > max_dis_line)
          {
            max_dis_line = fabs(temp_dis_line);
            corner_point = temp_seed_point;
          }
          seed_point_vector.push_back(temp_seed_point);
          if(0)//可视化
          {
            CvPoint temp_point = cvPointFrom32f(temp_seed_point);
//            ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 0]=0; // B
//            ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 1]=0; // G
//            ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 2]=255; // R
            this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[0] = 0;
            this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[1] = 0;
            this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[2] = 255;
          }
        }
      }
    }

    if (1)
    {
      Line_s base_line = {0};
      InitLine(&base_line, start_point.x, start_point.y, end_point.x, end_point.y);
      float dis_line = fabs(corner_point.x*base_line.a + corner_point.y*base_line.b - base_line.c);

      CvPoint2D32f line_start = cvPoint2D32f(start_point.x - corner_point.x, start_point.y - corner_point.y);
      CvPoint2D32f line_end = cvPoint2D32f(end_point.x - corner_point.x, end_point.y - corner_point.y);
      float temp_angle = BaseFunction::Angle_line2(line_start, line_end) - 90;

      float temp_c_ego = ego_veh_position.x*base_line.a + ego_veh_position.y*base_line.b - base_line.c;
      float temp_c = corner_point.x*base_line.a + corner_point.y*base_line.b - base_line.c;

      float dis_corner_start = sqrt( ( corner_point.x-start_point.x)*(corner_point.x-start_point.x)
          + ( corner_point.y-start_point.y)*( corner_point.x-start_point.y));
      float dis_corner_end = sqrt( ( corner_point.x-end_point.x)*(corner_point.x-end_point.x)
          + ( corner_point.y-end_point.y)*( corner_point.y-end_point.y));

      fit_method = 1;
      if ( ((dis_line > 0.6/map_resolution &&temp_angle<35  &&temp_angle >-30 )
          || (temp_angle<15  &&temp_angle >-20 && dis_line > 0.3/map_resolution) )
          && dis_corner_end>0.3/map_resolution && dis_corner_start>0.3/map_resolution)//
      {
        fit_method = 2;//判断是L型目标
      }
      if(0)
      {
        CvPoint temp_end = cvPointFrom32f(end_point);
        cv::circle(this->img_result_show_, temp_end, 2, cvScalar(125,255,75), 1, 8,0);
        CvPoint temp_start = cvPointFrom32f(start_point);
        cv::circle(this->img_result_show_, temp_start, 2, cvScalar(255,255,0), 1, 8,0);
        CvPoint temp_corner = cvPointFrom32f(corner_point);
        cv::circle(this->img_result_show_, temp_corner, 3, cvScalar(0,255,0), 1, 8,0);
      }
    }
    delete[] object_virtual_line;
  }

  /*************
   * 开始边缘拟合
   ************/
  CvPoint2D32f base_point = corner_point;//距离最近点
  float fit_line_angle = temp_object->object_box.angle*pi/180;
  if (fit_method == 0)//不拟合
  {
    fit_line_angle = temp_object->object_box.angle*pi/180;
  }

  if (seed_point_vector.size()>0 && fit_method>0)
  {
    vector<CvPoint2D32f> temp_seed_vector;
    temp_seed_vector.clear();
    int temp_seed_num = 0;
    if (fit_method == 1)//直接拟合直线
    {
      for (int i = 0; i < seed_point_vector.size(); ++i)
      {
        CvPoint2D32f temp_seed_point = seed_point_vector.at(i);
        temp_seed_vector.push_back(temp_seed_point);
        temp_seed_num++;
        if(1)
        {
          CvPoint temp_point = cvPointFrom32f(temp_seed_point);
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 0]=0; // B
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 1]=255; // G
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 2]=0; // R
          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[0] = 0;
          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[1] = 255;
          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[2] = 0;
        }
      }
    }
    if (fit_method == 2)//分段拟合
    {
      float corner_angle = BaseFunction::Angle_atan2(corner_point.x - ego_veh_position.x, ego_veh_position.y - corner_point.y);

      vector<CvPoint2D32f> seed_line_vector1;//种子点分段1
      int seed_line_num1 = 0;
      int seed_line_num2 = 0;
      vector<CvPoint2D32f> seed_line_vector2;//分段2
      float dis_corner_seed1 = 0;
      float dis_corner_seed2 = 0;
      for (int i=0; i<seed_point_vector.size(); i++)
      {
        CvPoint2D32f temp_seed_point = seed_point_vector.at(i);
        float temp_dis = sqrt( ( corner_point.x-temp_seed_point.x)*(corner_point.x-temp_seed_point.x)
            + ( corner_point.y-temp_seed_point.y)*( corner_point.y-temp_seed_point.y));
        float temp_angle = BaseFunction::Angle_atan2(temp_seed_point.x - ego_veh_position.x, ego_veh_position.y - temp_seed_point.y);
        temp_angle = temp_angle - corner_angle;
        if(temp_angle<-180)
          temp_angle = temp_angle + 360;
        if(temp_angle > 180)
          temp_angle = temp_angle - 360;

        if (temp_angle <= 0)
        {
          seed_line_num1++;
          seed_line_vector1.push_back(temp_seed_point);
          dis_corner_seed1 = max(dis_corner_seed1, temp_dis);
        }
        if (temp_angle >= 0)
        {
          seed_line_num2++;
          seed_line_vector2.push_back(temp_seed_point);
          dis_corner_seed2 = max(dis_corner_seed2, temp_dis);
        }
      }

      float delta_length = dis_corner_seed1 - dis_corner_seed2 ;
      float max_dis_corner = max(dis_corner_seed1, dis_corner_seed2);
      //如果分段边长相差较大，取边长较长者
      if ( fabs(delta_length) > 1/map_resolution || max_dis_corner > 3.6/map_resolution)
      {
        if (dis_corner_seed1 > dis_corner_seed2)
        {
          for (int i = 0; i < seed_line_vector1.size(); ++i)
          {
            CvPoint2D32f temp_seed_point = seed_line_vector1.at(i);
            temp_seed_vector.push_back(temp_seed_point);
            temp_seed_num++;
          }
        } else
        {
          for (int i = 0; i < seed_line_vector2.size(); ++i)
          {
            CvPoint2D32f temp_seed_point = seed_line_vector2.at(i);
            temp_seed_vector.push_back(temp_seed_point);
            temp_seed_num++;
          }
        }
      }else//如果两个分段边长差不多,取种子点数多的进行拟合
      {
        if (seed_line_num1 > seed_line_num2)
        {
          for (int i = 0; i < seed_line_vector1.size(); ++i)
          {
            CvPoint2D32f temp_seed_point = seed_line_vector1.at(i);
            temp_seed_vector.push_back(temp_seed_point);
            temp_seed_num++;
          }
        } else
        {
          for (int i = 0; i < seed_line_vector2.size(); ++i)
          {
            CvPoint2D32f temp_seed_point = seed_line_vector2.at(i);
            temp_seed_vector.push_back(temp_seed_point);
            temp_seed_num++;
          }
        }
      }
    }//end if (fit_method == 2)//分段拟合

    //开始直线拟合
    CvPoint2D32f* seed_points = (CvPoint2D32f*)malloc( temp_seed_num * sizeof(seed_points[0]));
    for (int i=0; i<temp_seed_vector.size(); i++)
    {
      CvPoint2D32f temp_seed_point = temp_seed_vector.at(i);
      seed_points[i] = temp_seed_point;
      if(1)//绘制用于直线拟合的种子点
      {
        CvPoint temp_point = cvPointFrom32f(temp_seed_point);
//        ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 0]=0; // B
//        ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 1]=0; // G
//        ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 2]=255; // R
//
        this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[0] = 0;
        this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[1] = 0;
        this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[2] = 255;
      }
    }
    float fit_line_param[4];//fit_line_param[0]和fit_line_param[1]为直线方向,2,3代表线上的一个点
    memset(fit_line_param, 0 , 4*sizeof(float));
    CvMat pointMat = cvMat( 1, temp_seed_num, CV_32FC2, seed_points );
    cvFitLine( &pointMat, CV_DIST_L1, 1, 0.01, 0.01, fit_line_param );
    fit_line_angle = atan2(fit_line_param[1], fit_line_param[0]);//直线角度

    //可视化
    if(0)
    {
      char file_name_ID[50];
      sprintf(file_name_ID, "%d", fit_method);

      CvScalar word_color = cvScalar( 0, 0, 255 );
      CvPoint word_point;
      word_point.x = temp_object->object_rect.x + temp_object->object_rect.width/2;
      word_point.y = temp_object->object_rect.y;
      cv::putText(this->img_result_show_,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,word_color,1);
    }
    if(0)//绘制拟合直线
    {
      CvPoint2D32f temp_point_f = cvPoint2D32f(fit_line_param[2],fit_line_param[3]);
      temp_point_f.x = fit_line_param[2] + 2 - 15*cos(fit_line_angle);
      temp_point_f.y = fit_line_param[3]  + 2 - 15*sin(fit_line_angle);
      CvPoint temp_point1 = cvPointFrom32f(temp_point_f);
      CvPoint2D32f temp_point2_f;
      temp_point2_f.x = fit_line_param[2] + 2 + 15*cos(fit_line_angle);
      temp_point2_f.y = fit_line_param[3] + 2 + 15*sin(fit_line_angle);
      CvPoint temp_point2 = cvPointFrom32f(temp_point2_f);
      //cvCircle(img_result, temp_point1, 2, cvScalar(0,0,255), 2,8, 0);
      cv::line(this->img_result_show_, temp_point1, temp_point2, cvScalar(0,255,255), 1, 8,0);
    }
  }//end if (seed_point_vector.size()>0 && fit_method>0)

  //if (temp_object->object_type > 0)
  {
    CvPoint2D32f contour_point4[4];
    cvBoxPoints(temp_object->object_box, contour_point4);
    //给基准点赋值(距离最近栅格点)
    if (fit_method == 0)
      base_point = contour_point4[point_index0];
    if (fit_method == 2)
      base_point = corner_point;
    if (fit_method == 1)
    {
      base_point = end_point;
    }
    //将直线角度[-pi,pi]归一化到0-90度
    int temp_counter = 0;
    float temp_line_angle = fit_line_angle;
    while(temp_counter<5)
    {
      if (temp_line_angle < 0)
        temp_line_angle = temp_line_angle + pi/2;

      if (temp_line_angle > pi/2)
        temp_line_angle = temp_line_angle - pi/2;

      if (temp_line_angle >= 0 && temp_line_angle <= pi/2)
        break;
      temp_counter++;
    }
    fit_line_angle = temp_line_angle;
  }
  *line_angle_ =  fit_line_angle;
  *base_point_ = base_point;
}

void OGMDetector::CorrectShape(CandidateObject* temp_object, vector<CvPoint2D32f> grid_point_vector_,
    float line_angle_, CvPoint2D32f base_point_)
{
  CvPoint2D32f box_point4[4];
  CvPoint2D32f box_center = temp_object->center_point;
  cvBoxPoints(temp_object->object_box, box_point4);

  if (1)//修正CvBox2D object_box
  {
    CvPoint2D32f corner_point4[4];
    cvBoxPoints(temp_object->object_box, corner_point4);
    float line_length0 = temp_object->object_box.size.width;
    float line_length1 = temp_object->object_box.size.height;
    CvPoint2D32f temp_center_correct = temp_object->object_box.center;
    if (1)
    {
      Line_s fit_line4[4];//矩形框四个边直线方程
      memset( fit_line4, 0, 4*sizeof(Line_s) );
      fit_line4[0].angle = line_angle_;
      fit_line4[0].a = sin(fit_line4[0].angle);
      fit_line4[0].b = -cos(fit_line4[0].angle);
      fit_line4[2].angle = line_angle_;
      fit_line4[2].a = sin(fit_line4[2].angle);
      fit_line4[2].b = -cos(fit_line4[2].angle);

      fit_line4[1].angle = line_angle_ + pi/2;
      fit_line4[1].a = sin(fit_line4[1].angle);
      fit_line4[1].b = -cos(fit_line4[1].angle);
      fit_line4[3].angle = line_angle_ + pi/2;
      fit_line4[3].a = sin(fit_line4[3].angle);
      fit_line4[3].b = -cos(fit_line4[3].angle);

      float max_dis_line1 = -1000;
      float min_dis_line1 = 1000;
      float max_dis_line2 = -1000;
      float min_dis_line2 = 1000;
      CvPoint2D32f line_point4[4];
      for (int i=0; i<grid_point_vector_.size(); i++)//找到离两条拟合直线最远栅格点,存在line_point4[4]中
      {
        CvPoint2D32f grid_point = grid_point_vector_.at(i);

        if(0)
        {
          CvPoint temp_point = cvPointFrom32f(grid_point);
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 0]=0; // B
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 1]=0; // G
//          ((uchar *)(img_result->imageData + temp_point.y*img_result->widthStep))[temp_point.x*img_result->nChannels + 2]=255; // R

          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[0] = 0;
          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[1] = 0;
          this->img_result_show_.at<Vec3b>(temp_point.y, temp_point.x)[2] = 255;
        }
        //点到两条边直线的距离
        float dis_line_1 = (grid_point.x - box_center.x)*fit_line4[0].a + (grid_point.y - box_center.y)*fit_line4[0].b;
        float dis_line_2 = (grid_point.x - box_center.x)*fit_line4[1].a + (grid_point.y - box_center.y)*fit_line4[1].b;
        if (dis_line_1 < min_dis_line1)
        {
          min_dis_line1 = dis_line_1;
          line_point4[0] = grid_point;
        }
        if (dis_line_1 > max_dis_line1)
        {
          max_dis_line1 = dis_line_1;
          line_point4[2] = grid_point;
        }
        if (dis_line_2 < min_dis_line2)
        {
          min_dis_line2 = dis_line_2;
          line_point4[1] = grid_point;
        }
        if (dis_line_2 > max_dis_line2)
        {
          max_dis_line2 = dis_line_2;
          line_point4[3] = grid_point;
        }
      }
      //求出两条边截距
      fit_line4[0].c = (line_point4[0].x + 0.5) * fit_line4[0].a + (line_point4[0].y - 0.5) * fit_line4[0].b;
      fit_line4[2].c = (line_point4[2].x + 0.5) * fit_line4[2].a + (line_point4[2].y - 0.5) * fit_line4[2].b;
      float temp_width = fabs(max_dis_line1 - min_dis_line1);
      if (temp_width < 0.24/map_resolution)//直线只包含一个栅格,则扩大至1.2
      {
        float temp_c = (box_center.x + 0.5) * fit_line4[0].a + (box_center.y + 0.5) * fit_line4[0].b;
        fit_line4[0].c = temp_c - 0.6;
        fit_line4[2].c = temp_c + 0.6;
        temp_width =  1.2;
      }
      //求出两条边长度
      fit_line4[0].line_length = fabs(fit_line4[2].c - fit_line4[0].c);
      fit_line4[2].line_length = fabs(fit_line4[2].c - fit_line4[0].c);

      fit_line4[1].c = (line_point4[1].x + 0.5) * fit_line4[1].a + (line_point4[1].y - 0.5) * fit_line4[1].b;
      fit_line4[3].c = (line_point4[3].x + 0.5) * fit_line4[3].a + (line_point4[3].y - 0.5) * fit_line4[3].b;
      float temp_length = fabs(max_dis_line2 - min_dis_line2);
      if (temp_length < 0.24/map_resolution)
      {
        float temp_c = (box_center.x + 0.5) * fit_line4[1].a + (box_center.y + 0.5) * fit_line4[1].b;
        fit_line4[1].c = temp_c - 0.6;
        fit_line4[3].c = temp_c + 0.6;
        temp_length =  1.2;
      }
      fit_line4[1].line_length = fabs(fit_line4[1].c - fit_line4[3].c);
      fit_line4[3].line_length = fabs(fit_line4[1].c - fit_line4[3].c);



      line_length1 = temp_width + 2;//+2为了绘图可视化不遮挡栅格点
      line_length0 = temp_length + 2;
      //找到四条直线交点
      corner_point4[0] = Point_line2(fit_line4[0], fit_line4[1]);
      corner_point4[1] = Point_line2(fit_line4[1], fit_line4[2]);
      corner_point4[2] = Point_line2(fit_line4[2], fit_line4[3]);
      corner_point4[3] = Point_line2(fit_line4[3], fit_line4[0]);
      //根据交点求矩形中心点
      temp_center_correct.x = (corner_point4[0].x + corner_point4[2].x)/2;
      temp_center_correct.y = (corner_point4[0].y + corner_point4[2].y)/2;

      if (0)
      {
        CvPoint temp_point = cvPointFrom32f(corner_point4[0]);
        cv::circle(this->img_result_show_, temp_point, 1, cvScalar(0,255,0), 2, 8,0);
        temp_point = cvPointFrom32f(corner_point4[1]);
        cv::circle(this->img_result_show_, temp_point, 1, cvScalar(255,255,0), 2, 8,0);
        temp_point = cvPointFrom32f(corner_point4[2]);
        cv::circle(this->img_result_show_, temp_point, 1, cvScalar(255,0,0), 2, 8,0);
        temp_point = cvPointFrom32f(corner_point4[3]);
        cv::circle(this->img_result_show_, temp_point, 1, cvScalar(0,255,255), 2, 8,0);
      }

      if (0)
      {
        for (int i = 0; i < 4; ++i)
        {
          CvPoint temp_point = cvPointFrom32f(line_point4[i]);
          CvPoint temp_point1 ;
          temp_point1.x = temp_point.x + 20*cos(fit_line4[i].angle);
          temp_point1.y = temp_point.y + 20*sin(fit_line4[i].angle);
          CvPoint temp_point2 ;
          temp_point2.x = temp_point.x - 20*cos(fit_line4[i].angle);
          temp_point2.y = temp_point.y - 20*sin(fit_line4[i].angle);
          cv::line(this->img_result_show_, temp_point1, temp_point2, cvScalar(0,255,0), 1, 8,0);
        }
      }
    }


    CvBox2D temp_box = temp_object->object_box;
    if (1)
    {
      temp_box.angle = line_angle_*180/pi;
      temp_box.center = temp_center_correct;//修正的中心点
      temp_box.size.width = line_length0;//修正的边长
      temp_box.size.height = line_length1;
      temp_object->object_box = temp_box;
    }

  }

  if (1)//根据修正的object_box来修正矩形顶点，中心点,到车辆坐标系原点距离和方位角
  {
    CvPoint2D32f contour_point4[4];
    cvBoxPoints(temp_object->object_box, contour_point4);
    float temp_angle = temp_object->object_box.angle*pi/180;
    for (int m =0; m<4; m++)
    {
      int m_up = BaseFunction::value_in_threshod_int(m+1, 0, 3);
      temp_object->shape.point4[m].x = contour_point4[m].x;
      temp_object->shape.point4[m].y = contour_point4[m].y;

      float temp_x = contour_point4[m_up].x - contour_point4[m].x;
      float temp_y = contour_point4[m_up].y - contour_point4[m].y;
      float temp_length = sqrt(temp_x*temp_x + temp_y*temp_y);
      if (temp_length < 0.2/map_resolution)
        temp_length = 0.24/map_resolution;

      temp_object->shape.fit_line4[m].line_length = temp_length;
      temp_object->shape.fit_line4[m].angle = temp_angle + m*pi/2;
    }
    temp_object->center_point.x = temp_object->object_box.center.x;
    temp_object->center_point.y = temp_object->object_box.center.y;
    float temp_x = temp_object->center_point.x - ego_veh_position.x;
    float temp_y = ego_veh_position.y - temp_object->center_point.y;
    temp_object->dis_veh_xy = sqrt(temp_x*temp_x + temp_y*temp_y);
    temp_object->position_angle = BaseFunction::Angle_atan2(temp_x, temp_y);

    if(0)
    {
      char file_name_ID[50];
      sprintf(file_name_ID, "%.1f %.1f %.1f %.1f", temp_object->shape.fit_line4[0].line_length,
          temp_object->shape.fit_line4[1].line_length, temp_object->shape.fit_line4[2].line_length,
          temp_object->shape.fit_line4[3].line_length);

      CvScalar word_color = cvScalar( 0, 0, 255 );
      CvPoint word_point;
      word_point.x = temp_object->object_rect.x + temp_object->object_rect.width/2;
      word_point.y = temp_object->object_rect.y;
      cv::putText(this->img_result_show_,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.0,word_color,1);
    }
    if (0)
    {
      cv::Point point_vertex[4];
      for(int i=0; i<4; i++)
      {
        point_vertex[i].x = temp_object->shape.point4[i].x;
        point_vertex[i].y = temp_object->shape.point4[i].y;
      }
      int point_count = 4;
      const cv::Point* ppt = point_vertex;
      cv::polylines(this->img_result_show_, &ppt, &point_count, 1, 1, cvScalar(0,125,255), 1);
      CvPoint temp_base_point = cvPointFrom32f(base_point_);
      cv::circle(this->img_result_show_, temp_base_point, 1, cvScalar(255,125,0), 2, 8,0);
      CvPoint temp_center = cvPointFrom32f(temp_object->center_point);
      //cvCircle(img_result, temp_center, 4, cvScalar(0,0,255), 2, 8,0);
    }
  }
}

void OGMDetector::Trackpoint_index(CandidateObject* temp_object, int step_)
{
  int point_index0 = temp_object->shape.polar4[0].point_index;
  temp_object->track_point.x = temp_object->shape.point4[point_index0].x;
  temp_object->track_point.y = temp_object->shape.point4[point_index0].y;
  temp_object->track_index0 = temp_object->shape.polar4[0].point_index;
  temp_object->track_index1 = temp_object->shape.polar4[0].point_index;
}

void OGMDetector::OcclusionState(CandidateObject* temp_object, Polar_Cell* polar_ogm_)
{
  int point_index0 = temp_object->shape.polar4[0].point_index;
  int point_index1 = temp_object->shape.polar4[1].point_index;
  int point_index2 = temp_object->shape.polar4[2].point_index;
  int point_index3 = temp_object->shape.polar4[3].point_index;

  // occlusion ansys
  int line_index = temp_object->shape.polar4[1].angle/virtual_line_resolution;
  if (virtual_line_d[line_index].line_length > (temp_object->shape.polar4[1].dis_xy -0.7/map_resolution))
  {
    temp_object->shape.polar4[1].is_occluded = false;
    temp_object->shape.point4[point_index1].is_occluded = false;
  }
  else
  {
    temp_object->shape.point4[point_index1].is_occluded = true;
    temp_object->shape.polar4[1].is_occluded = true;
    temp_object->shape.polar4[1].occluded_disxy = virtual_line_d[line_index].line_length;
  }

  line_index = temp_object->shape.polar4[2].angle/virtual_line_resolution;
  if (virtual_line_d[line_index].line_length > (temp_object->shape.polar4[2].dis_xy -0.5/map_resolution))
  {
    temp_object->shape.polar4[2].is_occluded = false;
    temp_object->shape.point4[point_index2].is_occluded = false;
  }
  else
  {
    temp_object->shape.point4[point_index2].is_occluded = true;
    temp_object->shape.polar4[2].is_occluded = true;
    temp_object->shape.polar4[2].occluded_disxy = virtual_line_d[line_index].line_length;
  }
  if (temp_object->shape.polar4[0].point_index != temp_object->shape.polar4[1].point_index
      && temp_object->shape.polar4[0].point_index != temp_object->shape.polar4[2].point_index )
  {
    temp_object->shape.polar4[0].is_occluded = false;
    temp_object->shape.point4[point_index0].is_occluded = false;
    if (!temp_object->shape.polar4[1].is_occluded)
    {
      CvPoint2D32f point1, point01;
      point1.x = ego_veh_position. x - temp_object->shape.point4[point_index0].x;
      point1.y = ego_veh_position. y - temp_object->shape.point4[point_index0].y;
      point01.x = temp_object->shape.point4[point_index1].x - temp_object->shape.point4[point_index0].x;
      point01.y = temp_object->shape.point4[point_index1].y - temp_object->shape.point4[point_index0].y;
      float angle_line01 = BaseFunction::Angle_line2(point1, point01);
      if (angle_line01 < 155)
      {
        temp_object->shape.point4[point_index1].is_occluded = true;
        temp_object->shape.polar4[1].is_occluded = true;
      }
    }
    if (!temp_object->shape.polar4[2].is_occluded)
    {
      CvPoint2D32f point2, point02;
      point2.x = ego_veh_position. x - temp_object->shape.point4[point_index0].x;
      point2.y = ego_veh_position. y - temp_object->shape.point4[point_index0].y;
      point02.x = temp_object->shape.point4[point_index2].x - temp_object->shape.point4[point_index0].x;
      point02.y = temp_object->shape.point4[point_index2].y - temp_object->shape.point4[point_index0].y;
      float angle_line02 = BaseFunction::Angle_line2(point2, point02);
      if (angle_line02 < 155)
      {
        temp_object->shape.point4[point_index2].is_occluded = true;
        temp_object->shape.polar4[2].is_occluded = true;
      }
    }
  }

  if (!temp_object->shape.polar4[0].is_occluded)
  {
    if (!temp_object->shape.polar4[1].is_occluded && !temp_object->shape.polar4[2].is_occluded)//
      temp_object->occluded_state = 0;

    if (!temp_object->shape.polar4[1].is_occluded && temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 1;

    if (temp_object->shape.polar4[1].is_occluded && !temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 2;

    if (temp_object->shape.polar4[1].is_occluded && temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 4;
  }
  if (temp_object->shape.polar4[0].is_occluded)
  {
    if (!temp_object->shape.polar4[1].is_occluded && !temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 3;

    if (!temp_object->shape.polar4[1].is_occluded && temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 5;

    if (temp_object->shape.polar4[1].is_occluded && !temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 6;

    if (temp_object->shape.polar4[1].is_occluded && temp_object->shape.polar4[2].is_occluded)
      temp_object->occluded_state = 7;
  }

  // is entire width length
  if (temp_object->object_type>0 && temp_object->dis_veh_xy < 20/map_resolution)
  {
    bool is_entire_line0 = false;
    bool is_entire_line1 = false;
    bool is_entire_high = false;

    int line_index = (int)((temp_object->position_angle)/ virtual_line_resolution);
    if (temp_object->shape.polar4[0].point_index == temp_object->shape.polar4[2].point_index
        || temp_object->shape.polar4[0].point_index == temp_object->shape.polar4[1].point_index)
    {
      int point_index_sum = temp_object->shape.polar4[1].point_index + temp_object->shape.polar4[2].point_index;
      if (point_index_sum == 3)
        is_entire_line1 = true;

      if (point_index_sum != 3)
        is_entire_line0 = true;
    }

    if (temp_object->shape.polar4[0].point_index != temp_object->shape.polar4[1].point_index
        && temp_object->shape.polar4[0].point_index != temp_object->shape.polar4[2].point_index)
    {
      bool is_entire_line01 = false;
      bool is_entire_line02 = false;
      if (!temp_object->shape.polar4[0].is_occluded && !temp_object->shape.polar4[1].is_occluded)
        is_entire_line01 = 1;

      if (!temp_object->shape.polar4[0].is_occluded && !temp_object->shape.polar4[2].is_occluded)
        is_entire_line02 = true;

      int point_index_sum = temp_object->shape.polar4[0].point_index + temp_object->shape.polar4[1].point_index;
      if (point_index_sum == true)
      {
        is_entire_line0 = is_entire_line01;
        is_entire_line1 = is_entire_line02;
      }
      else
      {
        is_entire_line0 = is_entire_line02;
        is_entire_line1 = is_entire_line01;
      }
    }

    temp_object->shape.is_entire_line0 = is_entire_line0;
    temp_object->shape.is_entire_line1 = is_entire_line1;

    if(temp_object->dis_veh_xy < 15/map_resolution)
      temp_object->shape.is_entire_high = true;
  }
}

void OGMDetector::Target_Array()
{
  std::sort(total_object_vector.begin(),total_object_vector.end(),
      [](const CandidateObject& obj1,const CandidateObject& obj2){return obj1.position_angle<obj2.position_angle?true:false;});
}

void OGMDetector::TransfCandidate( const std::vector<CandidateObject>& object_image,std::vector<CandidateObject>& object_vehicle)
{
  int map_width = this->img_ogm_input_.cols; //401
  int map_height = this->img_ogm_input_.rows;//501
  CandidateObject object_ret;
  for(const auto temp_object:object_image) {
    object_ret = temp_object;//必须要这一步,防止有些属性在其他函数中被改变没有被传递过来
    object_ret.center_point.x = (temp_object.center_point.x - map_width/2)*map_resolution;
    object_ret.center_point.y = (map_height-1 -  map_offset_y - temp_object.center_point.y )*map_resolution;
    object_ret.track_point.x = (temp_object.track_point.x - map_width/2)*map_resolution;
    object_ret.track_point.y = (map_height-1 - temp_object.track_point.y - map_offset_y)*map_resolution;
    object_ret.dis_veh_xy = temp_object.dis_veh_xy*map_resolution;
    object_ret.match_state = 0;
    object_ret.match_index = -1;
    object_ret.point_index_pre2cd = 0;

    for(int m=0;m<4;++m) {
      object_ret.shape.point4[m].x = (temp_object.shape.point4[m].x - map_width/2)*map_resolution;
      object_ret.shape.point4[m].y = (map_height-1 - temp_object.shape.point4[m].y - map_offset_y)*map_resolution;
      object_ret.shape.polar4[m].dis_xy = temp_object.shape.polar4[m].dis_xy*map_resolution;
      object_ret.shape.polar4[m].occluded_disxy = temp_object.shape.polar4[m].occluded_disxy*map_resolution;

      object_ret.shape.fit_line4[m].angle = 2*pi - temp_object.shape.fit_line4[m].angle;//  TODO:角度为什么用2pi减
      object_ret.shape.fit_line4[m].a = sin(temp_object.shape.fit_line4[m].angle);
      object_ret.shape.fit_line4[m].b = -cos(temp_object.shape.fit_line4[m].angle);
      object_ret.shape.fit_line4[m].c = temp_object.shape.point4[m].x*temp_object.shape.fit_line4[m].a
          + temp_object.shape.point4[m].y*temp_object.shape.fit_line4[m].b;
      object_ret.shape.fit_line4[m].line_length = temp_object.shape.fit_line4[m].line_length*map_resolution;
    }
    object_ret.shape.line_length0 = temp_object.shape.fit_line4[0].line_length;
    object_ret.shape.line_length1 = temp_object.shape.fit_line4[1].line_length;

    object_ret.shape.cubic_model_3d.x_max = temp_object.shape.point4[2].x;
    object_ret.shape.cubic_model_3d.y_max = temp_object.shape.point4[1].y;
    object_ret.shape.cubic_model_3d.x_min = temp_object.shape.point4[0].x;
    object_ret.shape.cubic_model_3d.y_min = temp_object.shape.point4[3].y;
    object_vehicle.push_back(object_ret);
  }//end for(const auto temp_object:object_image)
}

vector<CandidateObject> OGMDetector::get_candidate_object_vector(bool do_transform)
{
  if(do_transform) {//图像坐标转车体坐标
    vector<CandidateObject> candidate_object_vec_res;
    this->TransfCandidate(total_object_vector, candidate_object_vec_res);
    return candidate_object_vec_res;
  }

  return this->total_object_vector;
}

void OGMDetector::Release()
{
  delete[] virtual_line_d;
  delete[] virtual_line;
  delete[] virtual_line_type;
}

bool OGMDetector::AllocateMemories(const cv::Mat& img_ogm)
{
  //初始化TargetDetecter成员变量
  virtual_line_d = new Line_int[virtual_line_num];
  BaseFunction::InitVirtualLine(virtual_line_d,virtual_line_num);

  virtual_line = new Line_int[virtual_line_num];
  BaseFunction::InitVirtualLine(virtual_line,virtual_line_num);

  virtual_line_type = new int[virtual_line_num];
  memset(virtual_line_type, 0, virtual_line_num*sizeof(int));

  //存储栅格地图
  this->img_ogm_origin_ = img_ogm.clone();
  cvtColor(img_ogm, this->img_result_show_,CV_GRAY2BGR);//转成三通道图,使用处理过的栅格图片
  this->img_color_ogm_ = this->img_result_show_.clone();
}

Mat OGMDetector::RemoveIsolatedPixels(const Mat& input_image)
{
  //先去除单个点
   cv::Mat input_image_compare;
   cv::bitwise_not(input_image,input_image_compare);
   cv::Mat kernel1 = (cv::Mat_<uchar>(3,3)<<0,0,0,
                                           0,1,0,
                                           0,0,0);
   cv::Mat kernel2 = (cv::Mat_<uchar>(3,3)<<1,1,1,
                                            1,0,1,
                                            1,1,1);
   cv::Mat hitormiss1;
   cv::morphologyEx(input_image,hitormiss1,cv::MORPH_ERODE, kernel1);
   cv::Mat hitormiss2;
   cv::morphologyEx(input_image_compare, hitormiss2,cv::MORPH_ERODE, kernel2);
   cv::Mat hitormiss;
   cv::bitwise_and(hitormiss1, hitormiss2,hitormiss);

   cv::Mat hitormiss_comp;
   cv::bitwise_not(hitormiss,hitormiss_comp);
   cv::bitwise_and(input_image, input_image, img_temp_,hitormiss_comp);
//   namedWindow("hole",CV_WINDOW_NORMAL);
//   imshow("hole",img_temp_);
   return img_temp_;
}

void OGMDetector::ShowDetectionResult(std::string window_name,cv::Mat& img_show, const vector<CandidateObject>& object_vector)
{
  ostringstream sstream;
  sstream<<"frame: "<<this->frame_counter_;//显示帧数
  cv::putText(img_show,sstream.str(),cv::Point(5, 35),cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  sstream.str("");//清空
  sstream<<"object: "<<object_vector.size();//显示检测目标数
  cv::putText(img_show,sstream.str(),cv::Point(5,65), cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  int temp_count = 0;
  for(const auto& temp_object:object_vector)//可视化检测目标
  {
    cv::Point pt_test4[4];
    for(int i=0; i<4; i++) {
      CvPoint2D32f point4 = temp_object.shape.point4[i];
      pt_test4[i] = cvPointFrom32f(point4);
    }
    const cv::Point* ppt = pt_test4;
    int point_count = 4;
    cv::polylines(img_show,&ppt,&point_count,1,true,cv::Scalar(0,125,255));//目标矩形框

    //显示目标序号
    sstream.str("");
    char file_name_ID[50];
    sprintf(file_name_ID, "%d", temp_count);
    sstream<<temp_count;
    cv::Point word_point(pt_test4[0].x+1, pt_test4[0].y);
    cv::putText(img_show,sstream.str(),word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,255,0),1);

    CvPoint track_pt = cvPointFrom32f(temp_object.track_point);
    CvPoint center_pt = cvPointFrom32f(temp_object.center_point);
    //      cvCircle(img_result, track_pt, 2, cvScalar(0,255,255), 2, 8);//跟踪点可视化
//    cv::circle(img_show,center_pt,2,cv::Scalar(0,255,0),2,8);
    ++temp_count;
  }

  //绘制本车
  dynamic_object_tracking::VisualizerUtils* vis_result = dynamic_object_tracking::VisualizerUtils::getInstance();
  vis_result->DrawEgoVehicle(img_show);
  cv::namedWindow(window_name,CV_WINDOW_NORMAL);
  cv::imshow(window_name,img_show);
  cv::waitKey(5);
}

void OGMDetector::DrawDetectionResult(cv::Mat& img_show, const vector<CandidateObject>& object_vector, bool draw_vehicle)
{
  dynamic_object_tracking::VisualizerUtils* vis_result = dynamic_object_tracking::VisualizerUtils::getInstance();

  char file_name_ID[50];
  sprintf(file_name_ID, "frame: %d", this->frame_counter_);//显示处理帧数
  CvPoint word_point = cvPoint(5, 35);
  cv::putText(img_show,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  memset(file_name_ID, 0, 50);
  sprintf(file_name_ID, "object: %d",object_vector.size());//显示检测目标数
  word_point = cvPoint(5, 65);
  cv::putText(img_show,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,1.4,cv::Scalar(255,255,0),2);

  int temp_count = 0;
  for(const auto& temp_object:object_vector){//遍历检测目标
    int point_count = 4;
    CvPoint pt_test4[4];
    CvPoint2D32f point4;
    //显示目标矩形框
    for(int i=0; i<4; i++){
      point4.x = temp_object.shape.point4[i].x;
      point4.y = temp_object.shape.point4[i].y;
      pt_test4[i] = cvPointFrom32f(point4);
    }
    CvPoint* ppt_test = pt_test4;
    cv::Point vertexPoints[1][4];
    vertexPoints[0][0] = pt_test4[0];
    vertexPoints[0][1] = pt_test4[1];
    vertexPoints[0][2] = pt_test4[2];
    vertexPoints[0][3] = pt_test4[3];
    const cv::Point* ppt[1] = {vertexPoints[0]};
    cv::polylines(img_show,ppt,&point_count,1,true,cv::Scalar(0,125,255));

    //显示目标序号
    char file_name_ID[50];
    sprintf(file_name_ID, "%d", temp_count);
    CvPoint word_point = cvPoint(pt_test4[0].x+1, pt_test4[0].y);
    cv::putText(img_show,file_name_ID,word_point,cv::FONT_HERSHEY_PLAIN,0.8,cv::Scalar(255,255,0),1);

    CvPoint track_pt = cvPointFrom32f(temp_object.track_point);
    CvPoint center_pt = cvPointFrom32f(temp_object.center_point);
    //      cvCircle(img_result, track_pt, 2, cvScalar(0,255,255), 2, 8);//跟踪点可视化
//    cv::circle(img_show,center_pt,2,cv::Scalar(0,255,0),2,8);//中心点可视化
    ++temp_count;
  }

  //绘制本车
  vis_result->DrawEgoVehicle(img_show);
}


void OGMDetector::getClusterdCloud(std::shared_ptr<std::vector<Cloud>> clusteredPoints,
                                 const vector<CandidateObject>& total_object_vector,
                                 const CloudPtr cloudIn,
                                 const std::vector<OGM_Cell>& rigid_ogm)
{
  if(img_ogm_origin_.type()!=CV_8UC1 || clusteredPoints == nullptr)
    throw std::logic_error("wrong img_ogm_origin_ type, should be CV_8UC1");

  cv::Mat img_show;
  cv::cvtColor(img_ogm_origin_,img_show,CV_GRAY2BGR);

  auto toPoint = [](const Point2D& shape_point)->cv::Point{return cv::Point(cvRound(shape_point.x),cvRound(shape_point.y));};

  int numCandidateObj = total_object_vector.size();
//  std::vector<pcl::PointCloud<pcl::PointXYZI>> clusteredPoints(numCandidateObj);//目标数量

  cv::Point vertexA,vertexB,vertexC,vertexD;
  vector<vector<cv::Point>> obj_rect_vertex(total_object_vector.size());
  //1)得到目标矩形框顶点坐标
  for(int obj = 0;obj<total_object_vector.size();++obj){//遍历所有矩形框
    vertexA = toPoint(total_object_vector.at(obj).shape.point4[0]);
    vertexB = toPoint(total_object_vector.at(obj).shape.point4[1]);
    vertexC = toPoint(total_object_vector.at(obj).shape.point4[2]);
    vertexD = toPoint(total_object_vector.at(obj).shape.point4[3]);
//    cout<<"obj: "<<obj<<endl;
//    cout<<vertexA<<" "<<vertexB<<" "<<vertexC<<" "<<vertexD<<endl;
    obj_rect_vertex[obj].push_back(vertexA);
    obj_rect_vertex[obj].push_back(vertexB);
    obj_rect_vertex[obj].push_back(vertexC);
    obj_rect_vertex[obj].push_back(vertexD);
  }
  //2)判断每个栅格点与矩形框关系
  for(int row = 0;row<img_ogm_origin_.rows;++row){
    for(int col = 0;col<img_ogm_origin_.cols;++col){
      //1)得到障碍物栅格
      if(img_ogm_origin_.at<uchar>(row,col)==0)
        continue;
      for(int obj = 0;obj<total_object_vector.size();++obj){//遍历所有矩形框
        if(BaseFunction::isPointInPolygon(col,row,obj_rect_vertex[obj])){//如果在某个目标矩形框内
//          cout<<"point in rect: "<<obj<<" "<<row<<" "<<col<<endl;
          int ogm_index = (img_ogm_origin_.rows-row-1)*img_ogm_origin_.cols + col;
          vector<int> cloud_idx = rigid_ogm.at(ogm_index).cloud_index;//得到点云索引序列
          for(const auto& point_idx:cloud_idx){
            (*clusteredPoints)[obj].push_back(cloudIn->points[point_idx]);//得到属于该占据栅格的所有点云
          }
          //可视化,同一个目标相同的颜色
          int r = (500*(obj+1))%255;
          int g = (100*(obj+1))%255;
          int b = (150*(obj+1))%255;
          img_show.at<cv::Vec3b>(row,col) = cv::Vec3b(b,g,r);
          break;
        }
      }
    }
  }

//  namedWindow("hhh",CV_WINDOW_NORMAL);
//  imshow("hhh",img_show);
//  return std::make_shared<std::vector<pcl::PointCloud<pcl::PointXYZI>> >(clusteredPoints);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr OGMDetector::makeClusteredCloud(const std::vector<Cloud>& cloudVec)
{
  // for visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  int count = 1;
  for(const auto& obj:cloudVec) {//遍历每个物体
    for(const auto& point:obj) {
      pcl::PointXYZRGB o;
      o.x = point.x;
      o.y = point.y;
      o.z = point.z;
      o.r = (500*count)%255;
      o.g = (100*count)%255;
      o.b = (150*count)%255;
      clusteredCloud->push_back(o);
    }
    ++count;
  }
  return clusteredCloud;
}

void OGMDetector::ruleBasedFilter(std::vector<pcl::PointCloud<pcl::PointXYZI>>& clusterdCloud,vector<CandidateObject>& total_object_vector,std::vector<ObjectInfo>& object_info_vec)
{
  if(clusterdCloud.size()!=total_object_vector.size()||clusterdCloud.size()!=object_info_vec.size())
    throw std::logic_error("OGMDetector::ruleBasedFilter clusterdCloud size != total_object_vector size");

//  cout<<"[INFO] clusterdCloud.size() "<<clusterdCloud.size()<<endl;
  ROS_WARN_STREAM("ruleBasedFilter input clustered cloud size is: "<<clusterdCloud.size());
  //目标滤除阈值
  float tLenMin = 0.5, tLenMax=5, tWidthMin = 0.25, tWidthMax = 3.8, tHeightMin = 0.5, tHeightMax = 2.6,tNumPoints = 50,
        tLWRatioMin = 1.0, tLWRatioMax = 3.0, tHLRatioMin = 0.1, tHLRatioMax = 2.0,tAreaMax = 20.0;
  float length,width,height,area,LWratio, HLratio,distance;

  int idx = 0, numPoints = 0;
  std::set<int> delete_idx_set;//目标删除集合
  for(const auto& obj:total_object_vector){//遍历聚类目标
    bool is_promising = false;
    numPoints = clusterdCloud[idx].size();
    distance = obj.dis_veh_xy*this->map_resolution;//目标距离

    if(distance <= 15) {
      if(numPoints < 200){
        delete_idx_set.insert(idx);
        object_info_vec[idx].numPoints = numPoints;
        object_info_vec[idx].isPromising = false;
        ++idx;
        continue;
      }
    }
    else{
      if(numPoints < tNumPoints){//点云数量条件
        //      cout<<"small numPoints idx: "<<idx<< " numPoints: "<<numPoints<<endl;
        delete_idx_set.insert(idx);
        object_info_vec[idx].numPoints = numPoints;
        object_info_vec[idx].isPromising = false;
        ++idx;
        continue;
      }
    }


    float x1 = obj.shape.point4[0].x;
    float y1 = obj.shape.point4[0].y;
    float x2 = obj.shape.point4[1].x;
    float y2 = obj.shape.point4[1].y;
    float x3 = obj.shape.point4[2].x;
    float y3 = obj.shape.point4[2].y;
    //得到物体扫描到的长和宽
    float dist1 = sqrt((x1-x2)*(x1-x2)+ (y1-y2)*(y1-y2))*this->map_resolution;
    float dist2 = sqrt((x3-x2)*(x3-x2)+ (y3-y2)*(y3-y2))*this->map_resolution;
    if(dist1 > dist2){
      length = dist1;
      width = dist2;
    }
    else{
      length = dist2;
      width = dist1;
    }

    pcl::PointXYZI min;//用于存放三个轴的最小值
    pcl::PointXYZI max;//用于存放三个轴的最大值
    pcl::getMinMax3D(clusterdCloud[idx],min,max);
    height = max.z - min.z;

//    cout<<"idx: "<<idx<< " numPoints "<<numPoints<<" length: "<<length<<" width: "<<width<<" height: "<<height<<endl;

    area = dist1*dist2;
    LWratio = length/width+0.05;
    HLratio = height/length;

    if(width > tWidthMin && width < tWidthMax){//宽度约束
      if(length > tLenMin && length < tLenMax){//长度约束
        if(height > tHeightMin && height < tHeightMax){//高度约束
          if(area < tAreaMax){//面积约束
            if(distance <= 20.0){
              if(LWratio > tLWRatioMin && LWratio < tLWRatioMax){//长宽比约束
                is_promising = true;
              }
            }
            else{
              if(HLratio > tHLRatioMin && HLratio < tHLRatioMax){//高长比约束,去掉树干等细长目标
                is_promising = true;
              }
            }
          }
        }
      }
    }
    if(!is_promising)
      delete_idx_set.insert(idx);//需要删除的目标

    object_info_vec[idx].numPoints = numPoints;
    object_info_vec[idx].length = length;
    object_info_vec[idx].width = width;
    object_info_vec[idx].height = height;
    object_info_vec[idx].area = area;
    object_info_vec[idx].HLratio = HLratio;
    object_info_vec[idx].isPromising = is_promising;

    ++idx;
  }

//  cout<<"-----------------delete------------------"<<endl;
//  cout<<"delete_idx_set size: "<<delete_idx_set.size()<<endl;
//  for(const auto& idx:delete_idx_set){
//    cout<<idx<<" ";
//  }
//  cout<<endl;

  //遍历索引序列,剔除不符合条件目标
  int idex = 0;
  for(auto obj_iter = total_object_vector.begin(); obj_iter != total_object_vector.end();++idex) {
    auto it = delete_idx_set.find(idex);
    if(it!=delete_idx_set.end()){//剔除目标
      obj_iter = total_object_vector.erase(obj_iter);//最终传出的目标序列
//      clusterdCloud.erase(clusterdCloud.begin() + i);//聚类目标点云序列
    }
    else
      ++obj_iter;
  }

  ROS_WARN_STREAM("ruleBasedFilter after rulefilter clustered cloud size is: "<<total_object_vector.size());
}

void OGMDetector::Detect(vector<Line_int>& boundary_line)
{
  //1)remove isolated pixels in input ogm image
  Mat isolated_img = this->RemoveIsolatedPixels(this->img_ogm_input_);
  this->AllocateMemories(isolated_img);

  vector<CandidateObject>().swap(this->total_object_vector);//清空检测的目标
  //2) main handler
  /**********************
   *    Main Handlers
   **********************/
  cv::Mat img_close_get = this->PreProcessing(this->img_ogm_origin_);//膨胀+形态学闭运算
  this->Virtual_line(img_close_get);          //画射线及射线聚类
  this->TargetContourClustering(img_close_get);
  this->TargetIdentify(this->rigid_ogm_vec_, this->polar_ogm_);//精细检测:边缘轮廓拟合,该步骤进行了一些目标的剔除

  //3) apply rule based filter
  std::shared_ptr<std::vector<Cloud>> clustered_cloud = std::make_shared<std::vector<Cloud>>(this->total_object_vector.size());
  this->getClusterdCloud(clustered_cloud,this->total_object_vector,this->cloud_input_,this->rigid_ogm_vec_);
  this->detect_objecs_info_ = vector<ObjectInfo>(this->total_object_vector.size());
  this->ruleBasedFilter(*clustered_cloud,this->total_object_vector, this->detect_objecs_info_);

  //4) allocate different color for different clusters
  //同一目标点云赋予相同颜色
  this->cloud_clustered_rgb_ = this->makeClusteredCloud(*clustered_cloud);

  this->clusterd_cloud_vec_ = clustered_cloud;
  Target_Array();//按中心点方位角顺时针排序所有矩形框
  /********************************
   *   得到最终检测到的目标序列传出
   ********************************/
  //将前面几个函数中绘制好的射线复制给指针boundary_line_传出
  for(int i = 0; i < boundary_line.size(); ++i) {
    boundary_line[i].x1 = virtual_line_d[i].x1;
    boundary_line[i].y1 = virtual_line_d[i].y1;
    boundary_line[i].x2 = virtual_line_d[i].x2 ;
    boundary_line[i].y2 = virtual_line_d[i].y2;
    boundary_line[i].line_length= virtual_line_d[i].line_length;
    boundary_line[i].angle= virtual_line_d[i].angle;
    boundary_line[i].type = virtual_line_d[i].type;
    boundary_line[i].left_invalid = virtual_line_d[i].left_invalid;
    boundary_line[i].right_invalid = virtual_line_d[i].right_invalid;
  }
  Release();
}

void OGMDetector::DrawResult(bool draw_detect_info)
{
  int map_width = this->img_ogm_input_.cols; //401
  int map_height = this->img_ogm_input_.rows;//501
  //未加剔除条件前可视化
//  cv::Mat img_show_no_filtered = cv::cvarrToMat(this->img_result,false);
  if(this->is_show_img) {
    this->DrawDetectionResult(this->img_result_show_,this->total_object_vector);
  }

  if(this->is_show_img && draw_detect_info) {
    ROS_WARN("if(this->is_show_img && draw_detect_info) IS TRUE");
    cv::Mat object_detection_map = cv::Mat::zeros(map_height,map_width + 250, CV_8UC3);
    //绘制一条分界线
    cv::line(object_detection_map,cv::Point(map_width,0),cv::Point(map_width, map_height - 1),cv::Scalar(122,122,122),2);
    //1)设置ROI,显示栅格地图及检测情况
    Mat img_roi =  object_detection_map(Rect(0,0,map_width,map_height));
    this->img_result_show_.copyTo(img_roi);
    //2)显示检测目标物理属性信息
    img_roi = object_detection_map.colRange(map_width,object_detection_map.cols);
    char info_str[50];
    for(int i = 0; i < this->detect_objecs_info_.size();++i){
      //图像左上方显示目标属性信息
      sprintf(info_str,"%d %d %.2f %.2f %.2f %.2f %.2f %d",
          i,
          this->detect_objecs_info_[i].numPoints,
          this->detect_objecs_info_[i].length,
          this->detect_objecs_info_[i].width,
          this->detect_objecs_info_[i].height,
          this->detect_objecs_info_[i].area,
          this->detect_objecs_info_[i].HLratio,
          this->detect_objecs_info_[i].isPromising);
      cv::Point word_point = cv::Point(5,25+i*10);
      cv::putText(img_roi, info_str, word_point, CV_FONT_HERSHEY_PLAIN, 0.8, cv::Scalar(255,0,255));//洋红色
    }
    namedWindow("img_result_d",CV_WINDOW_NORMAL);
    imshow("img_result_d",object_detection_map);
    waitKey(5);
  }

  /********************************
   *   可视化
   ********************************/
  if (0)//是否显示射线
  {
    for (int i=0; i<virtual_line_num; i++)
    {
      CvPoint temp_point1 = cvPoint(this->virtual_line_d[i].x1, this->virtual_line_d[i].y1);
      CvPoint temp_point2 = cvPoint(this->virtual_line_d[i].x2, this->virtual_line_d[i].y2);
      cv::line(this->img_result_show_, temp_point1, temp_point2, cvScalar(0,255,0),1,8,0);
    }
  }

}
