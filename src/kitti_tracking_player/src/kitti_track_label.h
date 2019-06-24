/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月6日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#ifndef SRC_KITTI_TRACKING_PLAYER_SRC_KITTI_TRACK_LABEL_H_
#define SRC_KITTI_TRACKING_PLAYER_SRC_KITTI_TRACK_LABEL_H_

#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
struct Geometry
{
  float length;
  float width;
  float height;
  float x;
  float y;
  float z;
  float ry;
};
struct ObjectDetect
{
  std::string type;
  int occluded;
  cv::Rect bbox;
  float alpha;
  Geometry geometric;
};

class KittiTrackLabel {
public:
  KittiTrackLabel(std::string full_filename_label02, cv::Size img_size);
  virtual ~KittiTrackLabel();

  std::vector<ObjectDetect> getObjectVec(int frame_num);
private:
  void readFileContent();
  void getObjectMap(const cv::Size& img_size);

  std::string full_filename_lable02_;
  std::vector<std::string> file_all_lines_;

  std::map<int, std::vector<ObjectDetect>> objMap_;
};

#endif /* SRC_KITTI_TRACKING_PLAYER_SRC_KITTI_TRACK_LABEL_H_ */
