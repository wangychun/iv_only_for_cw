#pragma once
#include "StructMovingTargetDefine.h"

namespace sensors_fusion {
namespace ogm_mapping {
////将点云投影直角坐标栅格,投影到向量
///**
// * @brief from input cloud get ogm statistic data
// * @param cloud_in[in]
// * @param ogm_out[out]
// * @param ogm_property[in]
// */
//void ProjectCloud2OGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property);
//
//void CreateOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property);

template <typename PointT>
void ProjectCloud2OGM(const pcl::PointCloud<PointT> cloud_in, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property)
{
  //栅格图像尺寸
  cv::Size map_size = ogm_property.mapSize();
  pcl::PointCloud<pcl::PointXYZI> filteredCloud;
  for (int i = 0; i < cloud_in.points.size(); ++i) {//遍历点云
    //获得每个点的属性
    float x = cloud_in.points[i].x;
    float y = cloud_in.points[i].y;
    float z = cloud_in.points[i].z;
    {
      float newy = y + ogm_property.ogm_y_offset_;
      if ( x >= -ogm_property.ogm_width_/2 && x < ogm_property.ogm_width_/2  && newy >= 0 && newy < ogm_property.ogm_height_)
      {
        int col = (int)((x + ogm_property.ogm_width_/2) / ogm_property.resolution_);
        int row = (int)(newy /  ogm_property.resolution_) ;
        int index = row * map_size.width + col;//栅格地图数组中索引
        if(row >=0 && row < map_size.height && col >=0 && col < map_size.width)
        {
          ogm_out[index].points_num++;
          ogm_out[index].min_z = std::min(z, ogm_out[index].min_z);
          ogm_out[index].max_z = std::max(z, ogm_out[index].max_z);
//          ogm_out[index].intensity = std::max(intensity, ogm_out[index].intensity);
          //一个栅格中的点云平均高度
          ogm_out[index].average_z = ( ogm_out[index].average_z*(ogm_out[index].points_num-1) + z )/ ogm_out[index].points_num;
          //即该帧点云中符合条件的点的最大索引值
          ogm_out[index].endpoint_index = std::max(i, ogm_out[index].endpoint_index);
          //即该帧点云中符合条件的点的最小索引值
          ogm_out[index].startpoint_index = std::min(i, ogm_out[index].startpoint_index);
          ogm_out[index].endlaser_index = 0;//max(laser_index, rigid_ogm_[index].endlaser_index);
          ogm_out[index].startlaser_index = 0;//min(laser_index, rigid_ogm_[index].startlaser_index);
          ogm_out[index].layers_num = ogm_out[index].endlaser_index -ogm_out[index].startlaser_index + 1;
          //一个栅格中点云的最大高度差
          ogm_out[index].delta_z = ogm_out[index].max_z - ogm_out[index].min_z;
          ogm_out[index].type = CellProperty::Unidentify;
        }
      }//end if ( x >= -ogm_width/2 && x < ogm_width/2  && newy >= 0 && newy < ogm_height)
    }
  }
}

template <typename PointT>
void CreateOGM(const pcl::PointCloud<PointT> cloudIn, std::vector<OGM_Cell>& ogm_out, const OGMProperty& ogm_property)
{
  vector<OGM_Cell>(ogm_property.mapCellNum()).swap(ogm_out);
  BaseFunction::InitOGM(ogm_out);
  ProjectCloud2OGM(cloudIn, ogm_out, ogm_property);
}

void ProjectCloud2PolarOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, vector<Polar_Cell>& polar_ogm_out, const PolarProperty& polar_property);

void CreatePolarOGM(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, vector<Polar_Cell>& polar_ogm_out, const PolarProperty& polar_property);
}
}

