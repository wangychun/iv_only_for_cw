/*
 * multi_object_tracking.cpp
 *
 *  Created on: 2018年6月6日
 *      Author: zhanghm
 */

#include "multi_object_tracking.h"
#include "detect/ground_removal.h"
#include "detect/component_clustering.h"
#include "detect/box_fitting.h"

#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
MultiObjectTracking::MultiObjectTracking():
elevatedCloud_(new pcl::PointCloud<pcl::PointXYZ>()),
groundCloud_(new pcl::PointCloud<pcl::PointXYZ>()),
clusteredCloud_ (new pcl::PointCloud<pcl::PointXYZRGB>)
{

}

MultiObjectTracking::~MultiObjectTracking() {
  // TODO Auto-generated destructor stub
}

void MultiObjectTracking::ProcessFrame()
{
  PreProcess();
}

void MultiObjectTracking::PreProcess()
{
  elevatedCloud_->clear();
  groundCloud_->clear();
  clusteredCloud_->clear();
  //1)地面点云剔除
  GroundCloudRemoval(cloud_input_, elevatedCloud_);
  //2)目标聚类
  // make array with initialized 0
  int numCluster = 0;
  array<array<int, numGrid>, numGrid> cartesianData{};
  componentClustering(elevatedCloud_, cartesianData, numCluster);//目标聚类
  // for visualization
  PointCloud<pcl::PointXYZRGB>::Ptr clusteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  makeClusteredCloud(elevatedCloud_, cartesianData, clusteredCloud_,numCluster);
  //3)边界拟合TODO:下面语句会在公路场景中出现程序崩溃,待查
//  bBoxes_ = boxFitting(elevatedCloud_, cartesianData, numCluster);//边界拟合
//  cout<<"numCluster "<<numCluster<<" "<<"bBoxes size is "<<bBoxes_.size()<<endl;

//  cv::Mat img_map = cv::Mat::zeros(200,200,CV_8UC3);
//  for(int cellX = 0; cellX < numGrid; cellX++){
//    for(int cellY = 0; cellY < numGrid; cellY++){
//      int row = 200 - cellY;
//      int col = cellX;
//      if(cartesianData[cellX][cellY] != 0){
//        img
//      }
//    }
//  }


}
