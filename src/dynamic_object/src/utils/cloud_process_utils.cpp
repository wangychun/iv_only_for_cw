/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年8月21日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include "cloud_process_utils.h"
using std::cout; using std::endl;
namespace sensors_fusion {

CloudTransformUtils::CloudTransformUtils() {
  // TODO Auto-generated constructor stub

}

CloudTransformUtils::~CloudTransformUtils() {
  // TODO Auto-generated destructor stub
}

void CloudTransformUtils::ApplyTransform(std::string lidarName, CloudPtr cloudIn, CloudPtr cloudOut)
{
  auto trans_params = this->extrinsic_params_.find(lidarName);
  if(trans_params == this->extrinsic_params_.end()){
    cout<<"[ERROR] Haven't initialize the "<<lidarName<<" transform parameters!!";
    return;
  }

  pcl::transformPointCloud(*cloudIn, *cloudOut,trans_params->second.get_transform_matrix());
}


CloudPair<pcl::PointXYZI> RemoveGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn, const OGMProperty& ogm_property, vector<OGM_Cell>& ogm_cells_vec, float heightTh)
{
  float ground_threshold = 0.15; //障碍物栅格中离最小z值以上0.15m以内认为是地面
  pcl::PointCloud<pcl::PointXYZI> cloud_no_ground, cloud_ground;//障碍物点云,地面点云
  for(int i = 0; i < cloudIn->size();++i) {
    float x = cloudIn->points[i].x;
    float y = cloudIn->points[i].y;
    float z = cloudIn->points[i].z;

    float newy = y + ogm_property.ogm_y_offset_;
    if (x >= -ogm_property.ogm_width_/2 && x < ogm_property.ogm_width_/2  && newy >= 0 && newy < ogm_property.ogm_height_) {
      int col = (int)((x + ogm_property.ogm_width_/2) / ogm_property.resolution_);
      int row = (int)(newy / ogm_property.resolution_) ;
      int index = row * ogm_property.mapSize().width + col;//栅格地图数组中索引
      if(ogm_cells_vec[index].delta_z <= heightTh) {//地面点
        ogm_cells_vec[index].type = CellProperty::Ground;
        cloud_ground.push_back(cloudIn->points[i]);
        continue;
      }
      else {//认为是障碍物点
        //        if(ogm_cells_vec[index].points_num>10)
        if(ogm_cells_vec[index].points_num > 5) {
          ogm_cells_vec[index].type = CellProperty::RigidNoPassable;//障碍物栅格
          if(z > (ogm_cells_vec[index].min_z + ground_threshold)){//去除掉其中的部分地面点云
            cloud_no_ground.push_back(cloudIn->points[i]);
            int cloud_idx = (int)cloud_no_ground.size() - 1;
            ogm_cells_vec[index].cloud_index.push_back(cloud_idx);//得到障碍物栅格对应的点云索引
          }
        }
      }
    }
  }//for(int i = 0; i < filterCloud->size();++i)
  return CloudPair<pcl::PointXYZI>(cloud_no_ground.makeShared(), cloud_ground.makeShared());
}

} /* namespace sensors_fusion */
