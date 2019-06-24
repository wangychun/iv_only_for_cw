
// #include <pcl/visualization/cloud_viewer.h>
//#include <iostream>
//#include <math.h>
//#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
//#include <array> // std::array
//#include"Eigen/Core"

#include "ground_removal.h"
#include "gaus_blur.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

//int numChannel = 80;
//int numBin = 120;
//const int numMedianKernel = 1;
float rMin = 3.4;
float rMax = 120;
//const float tHmin = -2.15;
// float tHmin = -2.0;
float tHmin = -0.2;
// float tHmax = -1.4;
float tHmax = 0.73;

float tHDiff = 0.4;
float hSeonsor = 2.0242;

Cell::Cell(){
    minZ = 1000;
    isGround = false;
}

void Cell::updateMinZ(float z) {
    if (z < minZ) minZ = z;
}



void filterCloud(PointCloud<PointXYZ> cloud, PointCloud<PointXYZ> & filteredCloud){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].y;
        float y = -cloud.points[i].x;
        float z = cloud.points[i].z;

        float distance = sqrt(x * x + y * y);
        if(distance <= rMin || distance >= rMax) {
            continue; // filter out
        }
        else{
            pcl::PointXYZ o;
            o.x = x;
            o.y = y;
            o.z = z;
            filteredCloud.push_back(o);
        }
    }
}

void getCellIndexFromPoints(float x, float y, int& chI, int& binI){
    float distance = sqrt(x * x + y * y);
    //normalize
    float chP = (atan2(y, x) + M_PI) / (2 * M_PI);
    float binP = (distance - rMin) / (rMax - rMin);
    //index
    chI = floor(chP*numChannel);
    binI = floor(binP*numBin);
//    cout << "bin ind: "<<binI << " ch ind: "<<chI <<endl;
}

void createAndMapPolarGrid(PointCloud<PointXYZ> cloud,
                           array<array<Cell, numBin>, numChannel>& polarData ){
    for (int i = 0; i < cloud.size(); i++) {
        float x = cloud.points[i].x;
        float y = cloud.points[i].y;
        float z = cloud.points[i].z;

        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        // TODO; modify abobe function so that below code would not need
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue; // to prevent segentation fault
        polarData[chI][binI].updateMinZ(z);
    }
}

// update HDiff with larger value
void computeHDiffAdjacentCell(array<Cell, numBin>& channelData){
    for(int i = 0; i < channelData.size(); i++){
        // edge case
        if(i == 0){
            float hD = channelData[i].getHeight() - channelData[i+1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        else if(i == channelData.size()-1){
            float hD = channelData[i].getHeight() - channelData[i-1].getHeight();
            channelData[i].updateHDiff(hD);
        }
        // non-edge case
        else{
            float preHD  = channelData[i].getHeight() - channelData[i-1].getHeight();
            float postHD = channelData[i].getHeight() - channelData[i+1].getHeight();
            if(preHD > postHD) channelData[i].updateHDiff(preHD);
            else channelData[i].updateHDiff(postHD);
        }

//        cout <<channelData[i].getHeight() <<" " <<channelData[i].getHDiff() << endl;
    }
}

void applyMedianFilter(array<array<Cell, numBin>, numChannel>& polarData){
    // maybe later: consider edge case
    for(int channel = 1; channel < polarData.size()-1; channel++){
        for(int bin = 1; bin < polarData[0].size()-1; bin++){
            if(!polarData[channel][bin].isThisGround()){
                // target cell is non-ground AND surrounded by ground cells
                if(polarData[channel][bin+1].isThisGround()&&
                   polarData[channel][bin-1].isThisGround()&&
                   polarData[channel+1][bin].isThisGround()&&
                   polarData[channel-1][bin].isThisGround()){
                    vector<float> sur{polarData[channel][bin+1].getHeight(),
                                      polarData[channel][bin-1].getHeight(),
                                      polarData[channel+1][bin].getHeight(),
                                      polarData[channel-1][bin].getHeight()};
                    sort(sur.begin(), sur.end());
                    float m1 = sur[1]; float m2 = sur[2];
                    float median = (m1+m2)/2;
                    polarData[channel][bin].updataHeight(median);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}

void outlierFilter(array<array<Cell, numBin>, numChannel>& polarData){
    for(int channel = 1; channel < polarData.size() - 1; channel++) {
        for (int bin = 1; bin < polarData[0].size() - 2; bin++) {
            if(polarData[channel][bin].isThisGround()&&
               polarData[channel][bin+1].isThisGround()&&
               polarData[channel][bin-1].isThisGround()&&
               polarData[channel][bin+2].isThisGround()){
                float height1 = polarData[channel][bin-1].getHeight();
                float height2 = polarData[channel][bin].getHeight();
                float height3 = polarData[channel][bin+1].getHeight();
                float height4 = polarData[channel][bin+2].getHeight();
                if(height1 != tHmin && height2 == tHmin && height3 != tHmin){
                    float newH = (height1 + height3)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
                else if(height1 != tHmin && height2 == tHmin && height3 == tHmin && height4 != tHmin){
                    float newH = (height1 + height4)/2;
                    polarData[channel][bin].updataHeight(newH);
                    polarData[channel][bin].updateGround();
                }
            }
        }
    }
}


void groundRemove(PointCloud<pcl::PointXYZ>   cloud,
              PointCloud<pcl::PointXYZ>::Ptr  elevatedCloud,
              PointCloud<pcl::PointXYZ>::Ptr  groundCloud)
{

    PointCloud<pcl::PointXYZ> filteredCloud;

    filterCloud(cloud, filteredCloud);//filter too close or too long cloud
    array<array<Cell, numBin>, numChannel> polarData;
    createAndMapPolarGrid(filteredCloud, polarData);//得到一副存储了最小Z值的极坐标栅格地图

    // cout << "size: "<<groundCloud->size() << endl;
    for (int channel = 0; channel < polarData.size(); channel++)//row,80, 周向
    {
        for (int bin = 0; bin < polarData[0].size(); bin ++)//col,120, 径向
        {
            float zi = polarData[channel][bin].getMinZ();
            if(zi > tHmin && zi < tHmax){polarData[channel][bin].updataHeight(zi);}
            else if(zi > tHmax){polarData[channel][bin].updataHeight(hSeonsor);}
            else {polarData[channel][bin].updataHeight(tHmin);}
        }
        //could replace gauss with gradient
//        computeGradientAdjacentCell(polarData[channel]);
        gaussSmoothen(polarData[channel], 1, 3);
//        std::cout << " finished smoothing at channel "<< channel << std::endl;
        computeHDiffAdjacentCell(polarData[channel]);

        for (int bin = 0; bin < polarData[0].size(); bin ++){//径向遍历
            if(polarData[channel][bin].getSmoothed() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
            else if(polarData[channel][bin].getHeight() < tHmax &&
                    polarData[channel][bin].getHDiff() < tHDiff){
                polarData[channel][bin].updateGround();
            }
        }
    }
    // implement MedianFilter
    applyMedianFilter(polarData);
    // smoothen spot with outlier
    outlierFilter(polarData);

    for(int i = 0; i < filteredCloud.size(); i++) {
        float x = filteredCloud.points[i].x;
        float y = filteredCloud.points[i].y;
        float z = filteredCloud.points[i].z;

        pcl::PointXYZ o;
        o.x = x;
        o.y = y;
        o.z = z;
        int chI, binI;
        getCellIndexFromPoints(x, y, chI, binI);
        // assert(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin);
        if(chI < 0 || chI >=numChannel || binI < 0 || binI >= numBin) continue;
        
        if (polarData[chI][binI].isThisGround()) {
            float hGround = polarData[chI][binI].getHGround();
            if (z < (hGround + 0.25)) {
                groundCloud->push_back(o);//obtain ground cloud
            } else {
                elevatedCloud->push_back(o);//in ground grid, also have non-ground cloud points
            }
        } else {
            elevatedCloud->push_back(o);
        }
    }
}

bool get_image_coord(const float& xv,const float& yv,int& row,int& col)
{
  //计算在图像位置
  //栅格地图预先设置好
  col = (int)((xv + 80/2) / 0.2);
  row = 501 - 1 - (int)((yv + 50) / 0.2);
  if(col < 0||col>=401||row<0||row>=501)
    return false;
  return true;//在图像范围内,有效
}

void GroundCloudRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground)//生成地面栅格
{
  float height_threshold = 0.35;
  float ground_threshold = 0.15; //障碍物栅格中离最小z值以上0.15m以内认为是地面
  //存栅格内点云最小高度
  cv::Mat min_z_map = 1000.0*cv::Mat::ones(501,401,CV_32FC1);
  cv::Mat max_z_map = -1000.0*cv::Mat::ones(501,401,CV_32FC1);
  cv::Mat delta_z_map = cv::Mat::zeros(501,401,CV_32FC1);
  for(int i = 0 ;i < cloud_in->size();++i)
  {
    float x = cloud_in->points[i].x;
    float     y = cloud_in->points[i].y;
    float     z = cloud_in->points[i].z;
    //1)去除车体周围点云及噪点
    if(sqrt(pow(x,2)+pow(y,2))<3.0||(z<-5.0))
      continue;
    //2)形成一张最小z值栅格地图
    //计算在图像位置
    int row,col;
    if(!get_image_coord(x,y,row,col))
      continue;
    if(min_z_map.at<float>(row,col)>z)
      min_z_map.at<float>(row,col) = z;
    if(max_z_map.at<float>(row,col)<z)
      max_z_map.at<float>(row,col) = z;
    delta_z_map.at<float>(row,col) = max_z_map.at<float>(row,col) - min_z_map.at<float>(row,col);
  }
  //根据最小z值,得到去除地面点云
  for(int i = 0 ;i < cloud_in->size();++i)
  {
    float x = cloud_in->points[i].x;
    float     y = cloud_in->points[i].y;
    float     z = cloud_in->points[i].z;
    //1)去除车体周围点云及噪点
     if(sqrt(pow(x,2)+pow(y,2))<3.0||(z<-5.0))
       continue;
     int row,col;
     if(!get_image_coord(x,y,row,col))
       continue;
     if(delta_z_map.at<float>(row,col)<height_threshold)
     {
       continue;
     }
     else//认为是障碍物点
     {
       if(z > (min_z_map.at<float>(row,col)+ground_threshold))
         cloud_no_ground->push_back(cloud_in->points[i]);
     }
  }
}
