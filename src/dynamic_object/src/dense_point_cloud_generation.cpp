/*
 * dense_point_cloud_generation.cpp
 *
 *  Created on: 2018年5月23日
 *      Author: zhanghm
 */

#include "dense_point_cloud_generation.h"

DensePointCloudGeneration::DensePointCloudGeneration():current_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
dense_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
{

}

DensePointCloudGeneration::~DensePointCloudGeneration() {
  // TODO Auto-generated destructor stub
}

void DensePointCloudGeneration::setCloudWithPose(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_get,const transform::Rigid3d& transform_get)
{
  static transform::Rigid3d transform_temp = transform_get;
  Eigen::Quaterniond roatation_new = (transform_temp.rotation()).inverse()*transform_get.rotation();
  Eigen::Vector3d translation_new = transform_get.translation() - transform_temp.translation();
  transform::Rigid3d transform_relative(translation_new,roatation_new);
  current_cloud_ = cloud_get;//获得当前帧点云(局部坐标系)
  //查看点云队列
  if(cloud_queue_.size()>=TotalCloudNum)
  {
    cloud_queue_.pop_front();
    transform_queue_.pop_front();
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*cloud_get,*tempcloud,transform_get.translation(),transform_get.rotation());
  cloud_queue_.push_back(tempcloud);
  transform_queue_.push_back(transform_get);
}

void DensePointCloudGeneration::UpdateCloud()
{
  dense_cloud_->clear();//清空之前的所有点云

  current_transform_ = transform_queue_.back();
  transform::Rigid3d current_transorm_inverse = current_transform_.inverse();
  int count = 0;
  for(std::list<pcl::PointCloud<pcl::PointXYZI>::Ptr>::iterator it = cloud_queue_.begin();it!=cloud_queue_.end();++it)
  {
    ++ count;
    pcl::PointCloud<pcl::PointXYZI> tempcloud;
    //将全局坐标系下的历史点云转换到当前坐标系下
    pcl::transformPointCloud(*(*it),tempcloud,current_transorm_inverse.translation()
        ,current_transorm_inverse.rotation());
    (*dense_cloud_) +=tempcloud;//全都放在当前帧点云中形成多帧融合点云
//    if(count == cloud_queue_.size()-1)
//      break;
  }
//  (*dense_cloud_) += (*current_cloud_);

}

pcl::PointCloud<pcl::PointXYZI>::Ptr DensePointCloudGeneration::getDenseCloud()
{
  return dense_cloud_;
}
