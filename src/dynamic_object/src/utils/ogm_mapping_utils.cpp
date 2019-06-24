#include "utils/ogm_mapping_utils.h"
#include "StructMovingTargetDefine.h"
namespace sensors_fusion {
namespace ogm_mapping {
//
////将点云投影直角坐标栅格,投影到向量
//void ProjectCloud2OGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property)
//{
//  //栅格图像尺寸
//  cv::Size map_size = ogm_property.mapSize();
//  pcl::PointCloud<pcl::PointXYZI> filteredCloud;
//  for (int i = 0; i < cloud_in->points.size(); ++i) {//遍历点云
//    //获得每个点的属性
//    float x = cloud_in->points[i].x;
//    float y = cloud_in->points[i].y;
//    float z = cloud_in->points[i].z;
//    float intensity = cloud_in->points[i].intensity;
//    {
//      float newy = y + ogm_property.ogm_y_offset_;
//      if ( x >= -ogm_property.ogm_width_/2 && x < ogm_property.ogm_width_/2  && newy >= 0 && newy < ogm_property.ogm_height_)
//      {
//        int col = (int)((x + ogm_property.ogm_width_/2) / ogm_property.resolution_);
//        int row = (int)(newy /  ogm_property.resolution_) ;
//        int index = row * map_size.width + col;//栅格地图数组中索引
//        if(row >=0 && row < map_size.height && col >=0 && col < map_size.width)
//        {
//          ogm_out[index].points_num++;
//          ogm_out[index].min_z = std::min(z, ogm_out[index].min_z);
//          ogm_out[index].max_z = std::max(z, ogm_out[index].max_z);
//          ogm_out[index].intensity = std::max(intensity, ogm_out[index].intensity);
//          //一个栅格中的点云平均高度
//          ogm_out[index].average_z = ( ogm_out[index].average_z*(ogm_out[index].points_num-1) + z )/ ogm_out[index].points_num;
//          //即该帧点云中符合条件的点的最大索引值
//          ogm_out[index].endpoint_index = std::max(i, ogm_out[index].endpoint_index);
//          //即该帧点云中符合条件的点的最小索引值
//          ogm_out[index].startpoint_index = std::min(i, ogm_out[index].startpoint_index);
//          ogm_out[index].endlaser_index = 0;//max(laser_index, rigid_ogm_[index].endlaser_index);
//          ogm_out[index].startlaser_index = 0;//min(laser_index, rigid_ogm_[index].startlaser_index);
//          ogm_out[index].layers_num = ogm_out[index].endlaser_index -ogm_out[index].startlaser_index + 1;
//          //一个栅格中点云的最大高度差
//          ogm_out[index].delta_z = ogm_out[index].max_z - ogm_out[index].min_z;
//          ogm_out[index].type = CellProperty::Unidentify;
//        }
//      }//end if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
//    }
//  }
//}
//
//void CreateOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property)
//{
//  vector<OGM_Cell>(ogm_property.mapCellNum()).swap(ogm_out);
//  BaseFunction::InitOGM(ogm_out);
//  sensors_fusion::ogm_mapping::ProjectCloud2OGM(cloudIn, ogm_out, ogm_property);
//}

void ProjectCloud2PolarOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
    vector<Polar_Cell>& polar_ogm_out, const PolarProperty& polar_property)
{
  BaseFunction::InitPolarOGM(polar_ogm_out);
  for (int i = 0; i < cloudIn->points.size(); i++) {
    int point_id = i;
    float x = cloudIn->points[point_id].x;
    float y = cloudIn->points[point_id].y;
    float z = cloudIn->points[point_id].z;
    float intensity = cloudIn->points[point_id].intensity;

    float azimuth = BaseFunction::Angle_atan2(x, y);
    float dis_xy = sqrt(x*x + y*y);

    if ( !(x<1.5 && x>-1.5 && y<4.5 && y>-10)&& azimuth >= 0 &&
        azimuth < 360 && dis_xy>1.0 && dis_xy<polar_property.radial_limit_)
    {
      int angle_col = (int)(azimuth / polar_property.circular_resolution_);//周向
      int radius_row = (int)(dis_xy / polar_property.radial_resolution_);//径向

      if(radius_row >=0 && radius_row < polar_property.radialCellsNum() &&
          angle_col >=0 && angle_col < polar_property.circularCellsNum())
      {
        int index = radius_row * polar_property.circularCellsNum() + angle_col;
        polar_ogm_out[index].min_z = std::min(z, polar_ogm_out[index].min_z);
        polar_ogm_out[index].max_z = std::max(z, polar_ogm_out[index].max_z);
        polar_ogm_out[index].intensity = std::max(intensity, polar_ogm_out[index].intensity);

        polar_ogm_out[index].points_num++;
        polar_ogm_out[index].average_z = ( polar_ogm_out[index].average_z * (polar_ogm_out[index].points_num-1)
            + z)/ polar_ogm_out[index].points_num;
        polar_ogm_out[index].ground_xy = ( polar_ogm_out[index].ground_xy * (polar_ogm_out[index].points_num-1)
            + dis_xy )/ polar_ogm_out[index].points_num;

        polar_ogm_out[index].delta_z = polar_ogm_out[index].max_z - polar_ogm_out[index].min_z;
        polar_ogm_out[index].type = CellProperty::Unidentify;
      }
    }
  }
}

void CreatePolarOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, vector<Polar_Cell>& polar_ogm_out, const PolarProperty& polar_property)
{
  ProjectCloud2PolarOGM(cloudIn, polar_ogm_out, polar_property);
  //遍历极坐标栅格
  for (int j=0; j<polar_property.circularCellsNum(); j++) {
    for (int i = 6.5/polar_property.radial_resolution_; i < polar_property.radialCellsNum(); i++) {
      int polar_index = i*polar_property.circularCellsNum() + j;
      if( static_cast<int>(polar_ogm_out[polar_index].type) > static_cast<int>(CellProperty::Unknown ))
      {
        float deltaheight = (polar_ogm_out[polar_index].max_z - polar_ogm_out[polar_index].ground_z);
        if(i < 20/polar_property.radial_resolution_)
        {
          if (deltaheight>0.5 && polar_ogm_out[polar_index].delta_z > 0.35)
          {
            polar_ogm_out[polar_index].type = CellProperty::RigidNoPassable;
          }
          else
          {
            polar_ogm_out[polar_index].type = CellProperty::Ground;
          }
        }
        else{
          if (deltaheight>0.7 || polar_ogm_out[i].delta_z > 0.35)
          {
            polar_ogm_out[polar_index].type = CellProperty::RigidNoPassable;
          }
          else
          {
            polar_ogm_out[polar_index].type = CellProperty::Ground;
          }
        }
      }
    }
  }
}


}
}
//
//OGMMappingUtils::OGMMappingUtils()
//{
//}
//OGMMappingUtils::~OGMMappingUtils()
//{
//}
//
//void OGMMappingUtils::CloudLser2GridOGM( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_, int laserscanner_num_,
//    int startlaser_index_, int endlaser_index_, OGM_Cell* rigid_ogm_,
//    float ogmwidth_, float ogmheight_, float ogmresolution_, float ogm_offset_y_)
//{
//  float z_min = -2.0;
//  float z_max = 5.0;
//  //根据米制单位地图和分辨率计算栅格图像尺寸
//  int ogmwidth_cell_ = boost::math::round( ogmwidth_/ ogmresolution_)+1;
//  int ogmheight_cell_ = boost::math::round( ogmheight_/ ogmresolution_)+1;
//
//  for (int i = 0; i < cloud_all_->points.size(); ++i)//遍历点云
//  {
//    //获得每个点的属性
//    float x = cloud_all_->points[i].x;
//    float y = cloud_all_->points[i].y;
//    float z = cloud_all_->points[i].z;
//    float intensity = cloud_all_->points[i].intensity;
//
//    if (!(fabs(x)<1.5 && y<4.5&& y>-10) && z <z_max &&  z >z_min)//只考虑这个范围内的点云
//    {
//      float newy = y + ogm_offset_y_;
//      if ( x>=-ogmwidth_/2 && x<ogmwidth_/2  && newy>=0 && newy<ogmheight_)
//      {
//        int col = (int)((x + ogmwidth_/2) / ogmresolution_);
//        int row = (int)(newy / ogmresolution_) ;
//        int index = row * ogmwidth_cell_ + col;
//
//        if(row >=0 && row < ogmheight_cell_ && col >=0 && col < ogmwidth_cell_)
//        {
//          rigid_ogm_[index].min_z = std::min(z, rigid_ogm_[index].min_z);
//          rigid_ogm_[index].max_z = std::max(z, rigid_ogm_[index].max_z);
//          rigid_ogm_[index].intensity = std::max(intensity, rigid_ogm_[index].intensity);
//
//          rigid_ogm_[index].points_num++;
//          //一个栅格中的点云平均高度
//          rigid_ogm_[index].average_z = ( rigid_ogm_[index].average_z*(rigid_ogm_[index].points_num-1) + z )
//                                                              / rigid_ogm_[index].points_num;
//          //即该帧点云中符合条件的点的最大索引值
//          rigid_ogm_[index].endpoint_index = std::max(i, rigid_ogm_[index].endpoint_index);
//          //即该帧点云中符合条件的点的最小索引值
//          rigid_ogm_[index].startpoint_index = std::min(i, rigid_ogm_[index].startpoint_index);
//          rigid_ogm_[index].endlaser_index = 0;//max(laser_index, rigid_ogm_[index].endlaser_index);
//          rigid_ogm_[index].startlaser_index = 0;//min(laser_index, rigid_ogm_[index].startlaser_index);
//
//          rigid_ogm_[index].layers_num = rigid_ogm_[index].endlaser_index -
//              rigid_ogm_[index].startlaser_index + 1;
//          //一个栅格中点云的最大高度差
//          rigid_ogm_[index].delta_z = rigid_ogm_[index].max_z - rigid_ogm_[index].min_z;
//          rigid_ogm_[index].type = CellProperty::Unidentify;
//        }
//      }
//    }
//  }
//}
//
//pcl::PointCloud<pcl::PointXYZI>::Ptr OGMMappingUtils::ProjectCloud2OGM( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_,OGM_Cell* rigid_ogm_, float ogm_width,
//    float ogm_height, float ogm_resolution, float ogm_offset_y)
//{
//  //阈值选取 Unit: m
//  float x_limit_abs = 1.5;
//  float y_limit_forward1 = 5.0;
//  float y_limit_forward2 = 40;
//  float y_limit_back = -4.5;
//  float z_min = -2.0;
//  float z_max = 5.0;
//  //根据米制单位地图和分辨率计算栅格图像尺寸
//  int ogmwidth_cell_ = boost::math::round( ogm_width/ ogm_resolution)+1;
//  int ogmheight_cell_ = boost::math::round( ogm_height/ ogm_resolution)+1;
//
//  pcl::PointCloud<pcl::PointXYZI> filteredCloud;
//  for (int i = 0; i < cloud_all_->points.size(); ++i)//遍历点云
//  {
//    //获得每个点的属性
//    float x = cloud_all_->points[i].x;
//    float y = cloud_all_->points[i].y;
//    float z = cloud_all_->points[i].z;
//    float intensity = cloud_all_->points[i].intensity;
//
//    if(!(fabs(x) < x_limit_abs && y > y_limit_back && y < y_limit_forward1)&&
//        (y < y_limit_forward2)&& z < z_max && z > z_min)//只考虑这个范围内的点云
//    {
//      pcl::PointXYZI o;
//      o.x = x;
//      o.y = y;
//      o.z = z;
//      o.intensity = 0;
//      filteredCloud.push_back(o);
//
//      float newy = y + ogm_offset_y;
//      if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
//      {
//        int col = (int)((x + ogm_width/2) / ogm_resolution);
//        int row = (int)(newy / ogm_resolution) ;
//        int index = row * ogmwidth_cell_ + col;//栅格地图数组中索引
//        if(row >=0 && row < ogmheight_cell_ && col >=0 && col < ogmwidth_cell_)
//        {
//          rigid_ogm_[index].points_num++;
//          rigid_ogm_[index].min_z = std::min(z, rigid_ogm_[index].min_z);
//          rigid_ogm_[index].max_z = std::max(z, rigid_ogm_[index].max_z);
//          rigid_ogm_[index].intensity = std::max(intensity, rigid_ogm_[index].intensity);
//          //一个栅格中的点云平均高度
//          rigid_ogm_[index].average_z = ( rigid_ogm_[index].average_z*(rigid_ogm_[index].points_num-1) + z )/ rigid_ogm_[index].points_num;
//          //即该帧点云中符合条件的点的最大索引值
//          rigid_ogm_[index].endpoint_index = std::max(i, rigid_ogm_[index].endpoint_index);
//          //即该帧点云中符合条件的点的最小索引值
//          rigid_ogm_[index].startpoint_index = std::min(i, rigid_ogm_[index].startpoint_index);
//          rigid_ogm_[index].endlaser_index = 0;//max(laser_index, rigid_ogm_[index].endlaser_index);
//          rigid_ogm_[index].startlaser_index = 0;//min(laser_index, rigid_ogm_[index].startlaser_index);
//          rigid_ogm_[index].layers_num = rigid_ogm_[index].endlaser_index -rigid_ogm_[index].startlaser_index + 1;
//          //一个栅格中点云的最大高度差
//          rigid_ogm_[index].delta_z = rigid_ogm_[index].max_z - rigid_ogm_[index].min_z;
//          rigid_ogm_[index].type = CellProperty::Unidentify;
//        }
//      }//end if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
//    }
//  }
//  return filteredCloud.makeShared();
//}
//
//pcl::PointCloud<pcl::PointXYZI>::Ptr OGMMappingUtils::ProjectCloud2OGM( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_,std::vector<OGM_Cell>& rigid_ogm_, float ogm_width,
//    float ogm_height, float ogm_resolution, float ogm_offset_y)
//{
//  //根据米制单位地图和分辨率计算栅格图像尺寸
//  int ogmwidth_cell_ = boost::math::round( ogm_width/ ogm_resolution)+1;
//  int ogmheight_cell_ = boost::math::round( ogm_height/ ogm_resolution)+1;
//
//  pcl::PointCloud<pcl::PointXYZI> filteredCloud;
//  for (int i = 0; i < cloud_all_->points.size(); ++i)//遍历点云
//  {
//    //获得每个点的属性
//    float x = cloud_all_->points[i].x;
//    float y = cloud_all_->points[i].y;
//    float z = cloud_all_->points[i].z;
//    float intensity = cloud_all_->points[i].intensity;
//    {
//      pcl::PointXYZI o;
//      o.x = x;
//      o.y = y;
//      o.z = z;
//      o.intensity = 0;
//      filteredCloud.push_back(o);
//
//      float newy = y + ogm_offset_y;
//      if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
//      {
//        int col = (int)((x + ogm_width/2) / ogm_resolution);
//        int row = (int)(newy / ogm_resolution) ;
//        int index = row * ogmwidth_cell_ + col;//栅格地图数组中索引
//        if(row >=0 && row < ogmheight_cell_ && col >=0 && col < ogmwidth_cell_)
//        {
//          rigid_ogm_[index].points_num++;
//          rigid_ogm_[index].min_z = std::min(z, rigid_ogm_[index].min_z);
//          rigid_ogm_[index].max_z = std::max(z, rigid_ogm_[index].max_z);
//          rigid_ogm_[index].intensity = std::max(intensity, rigid_ogm_[index].intensity);
//          //一个栅格中的点云平均高度
//          rigid_ogm_[index].average_z = ( rigid_ogm_[index].average_z*(rigid_ogm_[index].points_num-1) + z )/ rigid_ogm_[index].points_num;
//          //即该帧点云中符合条件的点的最大索引值
//          rigid_ogm_[index].endpoint_index = std::max(i, rigid_ogm_[index].endpoint_index);
//          //即该帧点云中符合条件的点的最小索引值
//          rigid_ogm_[index].startpoint_index = std::min(i, rigid_ogm_[index].startpoint_index);
//          rigid_ogm_[index].endlaser_index = 0;//max(laser_index, rigid_ogm_[index].endlaser_index);
//          rigid_ogm_[index].startlaser_index = 0;//min(laser_index, rigid_ogm_[index].startlaser_index);
//          rigid_ogm_[index].layers_num = rigid_ogm_[index].endlaser_index -rigid_ogm_[index].startlaser_index + 1;
//          //一个栅格中点云的最大高度差
//          rigid_ogm_[index].delta_z = rigid_ogm_[index].max_z - rigid_ogm_[index].min_z;
//          rigid_ogm_[index].type = CellProperty::Unidentify;
//        }
//      }//end if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
//    }
//  }
//  return filteredCloud.makeShared();
//}
//
//void OGMMappingUtils::ProjectCloud2PolarOGM( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all_,
//    Polar_Cell* polar_ogm_, float polarogm_angle_,
//    float polarogm_radius_,float polarogm_angle_resolution_,
//    float polarogm_radius_resolution_ )
//{
//  int polarogm_angle_cell_ = boost::math::round( polarogm_angle_ / polarogm_angle_resolution_ );
//  int polarogm_radius_cell_ = boost::math::round( polarogm_radius_ / polarogm_radius_resolution_);
//
//  for (int i = 0; i < cloud_all_->points.size(); i++) {
//    int point_id = i;
//    float x = cloud_all_->points[point_id].x;
//    float y = cloud_all_->points[point_id].y;
//    float z = cloud_all_->points[point_id].z;
//    float intensity = cloud_all_->points[point_id].intensity;
//
//    float azimuth = BaseFunction::Angle_atan2(x, y);
//    float dis_xy = sqrt(x*x + y*y);
//
//    if ( !(x<1.5 && x>-1.5 && y<4.5 && y>-10)&& azimuth >= 0 &&
//        azimuth < 360 && dis_xy>1.0 && dis_xy<polarogm_radius_)
//    {
//      int angle_col = (int)(azimuth / polarogm_angle_resolution_);//周向
//      int radius_row = (int)(dis_xy / polarogm_radius_resolution_);//径向
//
//      if(radius_row >=0 && radius_row < polarogm_radius_cell_ &&
//          angle_col >=0 && angle_col < polarogm_angle_cell_)
//      {
//        int index = radius_row * polarogm_angle_cell_ + angle_col;
//        polar_ogm_[index].min_z = std::min(z, polar_ogm_[index].min_z);
//        polar_ogm_[index].max_z = std::max(z, polar_ogm_[index].max_z);
//        polar_ogm_[index].intensity = std::max(intensity, polar_ogm_[index].intensity);
//
//        polar_ogm_[index].points_num++;
//        polar_ogm_[index].average_z = ( polar_ogm_[index].average_z * (polar_ogm_[index].points_num-1)
//            + z)/ polar_ogm_[index].points_num;
//        polar_ogm_[index].ground_xy = ( polar_ogm_[index].ground_xy * (polar_ogm_[index].points_num-1)
//            + dis_xy )/ polar_ogm_[index].points_num;
////        polar_ogm_[index].endpoint_index = std::max(i, polar_ogm_[index].endpoint_index);
////        polar_ogm_[index].startpoint_index = std::min(i, polar_ogm_[index].startpoint_index);
////        polar_ogm_[index].endlaser_index = 0;
////        polar_ogm_[index].startlaser_index = 0;
//
//        polar_ogm_[index].delta_z = polar_ogm_[index].max_z - polar_ogm_[index].min_z;
////        polar_ogm_[index].layers_num = polar_ogm_[index].endlaser_index
////            - polar_ogm_[index].startlaser_index +1;
//        polar_ogm_[index].type = CellProperty::Unidentify;
//      }
//    }
//  }
//}
//
//void OGMMappingUtils::PolarOGM_Ground_first(Polar_Cell* polar_ogm_, Line_s* polar_fit_line_, float polar_angle_, float polar_radius_,
//    float polar_angle_resolution_, float polar_radius_resolution_,
//    OGM_Cell* rigid_ogm_, float ogmwidth_, float ogmheight_ , float ogmresolution_, float offest_y_ )
//{
//  int rigid_width_cell = boost::math::round(ogmwidth_ / ogmresolution_ )+1;
//  int rigid_height_cell = boost::math::round(ogmheight_ / ogmresolution_)+1;
//  int offfest_y = boost::math::round(offest_y_ / ogmresolution_);
//
//  int polar_angle_cell = boost::math::round(polar_angle_ / polar_angle_resolution_ );
//  int polar_radius_cell = boost::math::round(polar_radius_ / polar_radius_resolution_);
//  int polar_cell_size_ = polar_radius_cell* polar_angle_cell;
//
//  CvPoint ego_position = cvPoint(rigid_width_cell/2, offfest_y);
//  int polarogm_radius_first = boost::math::round( 20 / polar_radius_resolution_);
//
//  int* is_find_groundradius = new int[polar_angle_cell];
//  memset(is_find_groundradius, 0, polar_angle_cell*sizeof(int));
//
//  int* groundradius_index_first = new int[polar_angle_cell];
//  memset(groundradius_index_first, 0, polar_angle_cell*sizeof(int));
//  if (1)
//  {
//    ego_position.x = rigid_width_cell/2;
//    ego_position.y = offfest_y;
//    for (int i=ego_position.y - 5/ogmresolution_; i<(ego_position.y + 5/ogmresolution_); i++)
//    {
//      for (int j=ego_position.y - 5/ogmresolution_; j<(ego_position.y + 5/ogmresolution_); j++)
//      {
//        int rigid_index = i*rigid_width_cell + j;
//        if (rigid_ogm_[rigid_index].type == CellProperty::Unidentify)
//        {
//          if (rigid_ogm_[rigid_index].delta_z<0.35)
//          {
//            rigid_ogm_[rigid_index].type = CellProperty::Ground;
//          } else
//          {
//            rigid_ogm_[rigid_index].type =  CellProperty::RigidNoPassable;
//          }
//        }
//      }
//    }
//    int i_start = (int)(270/polar_angle_resolution_);
//    for (int ii = 0; ii < polar_angle_cell; ++ii)
//    {
//      int i = BaseFunction::value_in_threshod_int(i_start+ii, 0, polar_angle_cell-1);
//      float ground_z = 0;
//      int j_pre = 0;
//      int index_pre = j_pre * polar_angle_cell + i;
//      bool is_break = false;
//      for (int j = 2; j < 10 / polar_radius_resolution_;)
//      {
//        int polar_index = j * polar_angle_cell + i;
//        index_pre = j_pre * polar_angle_cell + i;
//        int j_up = j + 1;
//        int index_up = j_up * polar_angle_cell + i;
//        int temp_counter = 0;
//        bool is_up = false;
//        while (temp_counter < 6 && j_up < polar_radius_cell)
//        {
//          index_up = j_up * polar_angle_cell + i;
//          if (polar_ogm_[index_up].type == CellProperty::Unidentify)
//          {
//            is_up = true;
//            break;
//          }
//          temp_counter++;
//          j_up++;
//        }
//        if (polar_ogm_[polar_index].type == CellProperty::Unidentify)
//        {
//          ground_z = ground_z + polar_ogm_[index_pre].grad_m*(j-j_pre)*polar_radius_resolution_;
//          float delta_ground_z = polar_ogm_[polar_index].min_z - ground_z;
//          if (is_find_groundradius[i] > 0)
//          {
//            is_find_groundradius[i] = j;
//            ground_z = polar_ogm_[polar_index].min_z;
//            if (polar_ogm_[polar_index].delta_z < 0.35 && fabs(delta_ground_z)<0.35)
//              ground_z = polar_ogm_[polar_index].average_z;
//
//          } else
//          {
//            is_find_groundradius[i] = j;
//            ground_z = polar_ogm_[polar_index].min_z;
//            if(is_find_groundradius[i]>0)
//            {
//              polar_ogm_[polar_index].grad_b = j*polar_radius_resolution_;
//              polar_ogm_[polar_index].grad_m = ground_z;
//              for (int k = 1; k < is_find_groundradius[i]; ++k)
//              {
//                int temp_index1 = k*polar_angle_cell + i;
//                polar_ogm_[polar_index].ground_z = ground_z*k/j;
//                polar_ogm_[temp_index1].grad_b =  polar_ogm_[polar_index].grad_b;
//                polar_ogm_[temp_index1].grad_m = ground_z*k/j;
//              }
//            }
//          }
//
//          polar_ogm_[polar_index].grad_b = (j - j_pre)*polar_radius_resolution_;
//          polar_ogm_[polar_index].grad_m = (ground_z - polar_ogm_[index_pre].ground_z)/polar_ogm_[polar_index].grad_b;
//          polar_ogm_[polar_index].ground_z = ground_z;
//          is_find_groundradius[i] = j;
//          j_pre = j;
//
//          if(polar_ogm_[polar_index].delta_z < 0.35)
//          {
//            polar_ogm_[polar_index].type = CellProperty::Ground;
//          } else
//          {
//            polar_ogm_[polar_index].type = CellProperty::RigidNoPassable;
//            if (is_up)
//              polar_ogm_[polar_index].type = CellProperty::Passable;
//          }
//
//          if (!is_up)
//          {
//            break;
//          }
//          if (is_up)
//          {
//            float groun_z_up = ground_z + polar_ogm_[polar_index].grad_m*(j_up-j)*polar_angle_resolution_;
//            float delta_ground_z_up = polar_ogm_[index_up].min_z - groun_z_up;
//            float delta_z_up = polar_ogm_[index_up].min_z - polar_ogm_[polar_index].max_z;
//            float delta_z_sum = delta_ground_z_up + delta_z_up;
//            if (polar_ogm_[polar_index].delta_z > 0.5 && fabs(delta_ground_z_up) > 0.5)
//            {
//              break;
//            }
//          }
//        }
//        j = j_up;
//      }
//    }
//  }
//
//  delete[] groundradius_index_first;
//  delete[] is_find_groundradius;
//}
//
//void OGMMappingUtils::PolarOGM_Ground(Polar_Cell* polar_ogm_, Line_s* polar_fit_line_,float polarogm_angle_, float polarogm_radius_,
//    float polarogm_angle_resolution_, float polarogm_radius_resolution_,
//    Polar_Cell* largepolar_ogm_, float largepolarogm_angle_, float largepolarogm_radius_,
//    float largepolarogm_angle_resolution_, float largepolarogm_radius_resolution_)
//{
//  int polar_angle_cell = boost::math::round(polarogm_angle_ / polarogm_angle_resolution_ );
//  int polar_radius_cell = boost::math::round(polarogm_radius_ / polarogm_radius_resolution_);
//  int polar_cell_size_ = polar_radius_cell* polar_angle_cell;
//
//  int largepolar_angle_cell = boost::math::round(largepolarogm_angle_ / largepolarogm_angle_resolution_ );
//  int largepolar_radius_cell = boost::math::round(largepolarogm_radius_ / largepolarogm_radius_resolution_);
//  int largepolar_cell_size_ = largepolar_radius_cell* largepolar_angle_cell;
//
//
//
//}
//
//void OGMMappingUtils::AttachOGMProperties(Polar_Cell* polar_ogm_, float polarogm_angle_, float polarogm_radius_,
//    float polarogm_angle_resolution_, float polarogm_radius_resolution_)
//{
//  //可调阈值
//  int polarogm_radius_cell = boost::math::round( polarogm_radius_/ polarogm_radius_resolution_ );//径向
//  int polarogm_angle_cell = boost::math::round( polarogm_angle_/ polarogm_angle_resolution_ );//周向
//
//  //遍历极坐标栅格
//  for (int j=0; j<polarogm_angle_cell; j++) {
//    for (int i = 6.5/polarogm_radius_resolution_; i < polarogm_radius_cell; i++) {
//      int polar_index = i*polarogm_angle_cell + j;
//      if( static_cast<int>(polar_ogm_[polar_index].type) > static_cast<int>(CellProperty::Unknown ))
//      {
//        float deltaheight = (polar_ogm_[polar_index].max_z - polar_ogm_[polar_index].ground_z);
//        if(i < 20/polarogm_radius_resolution_)
//        {
//          if (deltaheight>0.5 && polar_ogm_[polar_index].delta_z > 0.35)
//          {
//            polar_ogm_[polar_index].type = CellProperty::RigidNoPassable;
//          }
//          else
//          {
//            polar_ogm_[polar_index].type = CellProperty::Ground;
//          }
//        }
//        else{
//          if (deltaheight>0.7 || polar_ogm_[i].delta_z > 0.35)
//          {
//            polar_ogm_[polar_index].type = CellProperty::RigidNoPassable;
//          }
//          else
//          {
//            polar_ogm_[polar_index].type = CellProperty::Ground;
//          }
//        }
//      }
//    }
//  }
//}
//
//void OGMMappingUtils::GridOGM_Ground(Polar_Cell* polar_ogm_, float polarogm_angle_, float polarogm_radius_,float polarogm_angle_resolution_, float polarogm_radius_resolution_,
//    OGM_Cell* rigid_ogm_, float ogm_width_, float ogm_height_ , float ogm_resolution_, float offest_y_ )
//{
//  int rigidogmwidth_cell = boost::math::round(ogm_width_ / ogm_resolution_) + 1;//501
//  int rigidogmheight_cell = boost::math::round(ogm_height_ / ogm_resolution_) + 1;//501
//  int polarogm_radius_cell = boost::math::round( polarogm_radius_/ polarogm_radius_resolution_ );
//  int polarogm_angle_cell = boost::math::round( polarogm_angle_/ polarogm_angle_resolution_ );
//  int polarogm_cell_size = polarogm_angle_cell * polarogm_radius_cell;
//
//  for (int j=0; j<polarogm_angle_cell; j++)
//  {
//    for (int i = 25/polarogm_radius_resolution_; i < polarogm_radius_cell; i++)
//    {
//      int polar_index = i*polarogm_angle_cell + j;
//      if( static_cast<int>(polar_ogm_[polar_index].type) > static_cast<int>(CellProperty::Unknown))
//      {
//        float deltaheight = (polar_ogm_[polar_index].max_z - polar_ogm_[polar_index].ground_z);
//        if(i < 20/polarogm_radius_resolution_)
//        {
//          if (deltaheight>0.5 && polar_ogm_[polar_index].delta_z > 0.35)
//          {
//            polar_ogm_[polar_index].type = CellProperty::RigidNoPassable;
//          }
//          else
//          {
//            polar_ogm_[polar_index].type = CellProperty::Ground;
//          }
//        } else
//        {
//          if (deltaheight>0.7 || polar_ogm_[i].delta_z > 0.35)
//          {
//            polar_ogm_[polar_index].type = CellProperty::RigidNoPassable;
//          }
//          else
//          {
//            polar_ogm_[polar_index].type = CellProperty::Ground;
//          }
//        }
//      }
//    }
//  }
//
//  for (int j = 0; j < rigidogmheight_cell; j++)
//  {
//    for (int i = 0;  i < rigidogmwidth_cell; ++ i)
//    {
//      int rigid_index = j*rigidogmwidth_cell + i;
//      if (static_cast<int>(rigid_ogm_[rigid_index].type) > static_cast<int>(CellProperty::Unknown))
//      {
//        float x = (i)*ogm_resolution_ -ogm_width_/2;
//        float y = j*ogm_resolution_ - offest_y_;
//        float azimuth = BaseFunction::Angle_atan2( x, y);
//        float dis_xy = sqrt(x*x + y*y);
//        rigid_ogm_[rigid_index].type = CellProperty::RigidNoPassable;
//      }
//    }
//  }
//}
