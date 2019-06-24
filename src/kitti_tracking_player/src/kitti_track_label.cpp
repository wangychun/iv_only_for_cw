/*======================================================================
 * Author   : Haiming Zhang
 * Email    : zhanghm_1995@qq.com
 * Version  :　2018年12月6日
 * Copyright    :
 * Descriptoin  :
 * References   :
 ======================================================================*/
#include "kitti_track_label.h"

#include <fstream>
#include <sstream>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/locale.hpp>
#include <boost/program_options.hpp>
#include <boost/progress.hpp>
#include <boost/tokenizer.hpp>

#include <ros/ros.h>
using namespace std;
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
KittiTrackLabel::KittiTrackLabel(std::string full_filename_label02, cv::Size img_size):
full_filename_lable02_(full_filename_label02)
{
  ROS_WARN_STREAM("full file name of label02 is "<<full_filename_label02);
  ROS_WARN_STREAM("image size is "<<img_size.width<< " "<<img_size.height);
  // TODO judge file exist or not
  // Read all content
  readFileContent();

  getObjectMap(img_size);
}

KittiTrackLabel::~KittiTrackLabel() {
}

std::vector<ObjectDetect> KittiTrackLabel::getObjectVec(int frame_num)
{
  // Find all bboxes in this image
  std::vector<ObjectDetect> rcs;
  std::map<int, std::vector<ObjectDetect>>::iterator it = objMap_.find(frame_num);
  if (it != objMap_.end()) {
    rcs = it->second;
  }
  return rcs;
}

void KittiTrackLabel::readFileContent()
{
  boost::char_separator<char> sep_line {"\n"};

  // Read all contents in file
  std::ifstream t(full_filename_lable02_);
  std::stringstream buffer;
  buffer << t.rdbuf();
  std::string contents(buffer.str());

  // Separate every line
  tokenizer tok_line(contents, sep_line);
  std::vector<std::string> lines(tok_line.begin(), tok_line.end());
  file_all_lines_ = lines;
}

void KittiTrackLabel::getObjectMap(const cv::Size& img_size)
{
  std::vector<ObjectDetect> objVec;
  int num = -1;
  int tmpNum = -1;

  // Loop all lines
  for(const auto& line : file_all_lines_) {
    boost::char_separator<char> sep {" "};

    tokenizer tok(line, sep);
    vector<string> cols(tok.begin(), tok.end());

    if(cols.size() < 17)
      continue;

    tmpNum = boost::lexical_cast<int>(cols[0]);
    if (num!=-1 && tmpNum!=num) {
      objMap_.insert(std::make_pair(num, objVec));
      objVec.clear();
      num = tmpNum;
    }
    if (num == -1) {
      num = tmpNum;
    }

    ObjectDetect obj;
    // Read type
    obj.type = cols[2];
    // Read occluded
    obj.occluded = boost::lexical_cast<int>(cols[4]);

    // Delete DontCare type and largely or unknown occluded objects
    if(obj.type == "DontCare" || obj.type == "Person_sitting" ||
        obj.occluded == 2 || obj.occluded == 3)
      continue;
    if(obj.type == "Van")
      obj.type = "Car";

    // Read alpha
    obj.alpha = boost::lexical_cast<float>(cols[5]);
    // Read bounding box
    cv::Rect rc;
    rc.x = (int)boost::lexical_cast<float>(cols[6]);
    rc.y = (int)boost::lexical_cast<float>(cols[7]);
    rc.width = (int)boost::lexical_cast<float>(cols[8]) - rc.x;
    rc.height = (int)boost::lexical_cast<float>(cols[9]) - rc.y;
    // check border conditions
    if(rc.x < 0)
      rc.x = 0;
    if(rc.y < 0)
      rc.y = 0;
    if(rc.br().x > img_size.width)
      rc.width = img_size.width - rc.x;
    if(rc.br().y > img_size.height)
      rc.height = img_size.height - rc.y;
    obj.bbox = rc;
    // Read dimensions
    obj.geometric.height = boost::lexical_cast<float>(cols[10]);
    obj.geometric.width = boost::lexical_cast<float>(cols[11]);
    obj.geometric.length = boost::lexical_cast<float>(cols[12]);
    obj.geometric.x = boost::lexical_cast<float>(cols[13]);
    obj.geometric.y = boost::lexical_cast<float>(cols[14]);
    obj.geometric.z = boost::lexical_cast<float>(cols[15]);
    obj.geometric.ry = boost::lexical_cast<float>(cols[16]);
    objVec.push_back(obj);
  }
  if (!objVec.empty()) {
    objMap_.insert(std::make_pair(tmpNum, objVec));
  }

}
