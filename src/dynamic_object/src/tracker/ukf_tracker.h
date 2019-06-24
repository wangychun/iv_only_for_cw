/*
 * ukf_tracker.h
 *
 *  Created on: 2018年6月28日
 *      Author: zhanghm
 */

#ifndef SRC_DYNAMIC_OBJECT_SRC_TRACKER_UKF_TRACKER_H_
#define SRC_DYNAMIC_OBJECT_SRC_TRACKER_UKF_TRACKER_H_
#include "../StructMovingTargetDefine.h"
class UKFTracker {
public:
  UKFTracker();
  virtual ~UKFTracker();
  void ProcessTracking(vector<CandidateObject>* candidat_object_vector_,
      vector<MovingObject>* moving_object_vector_,double time_stamp);
  void Correct_ContourPoint4(MovingObject *moving_object_, CandidateObject* candidate_object_);
};

#endif /* SRC_DYNAMIC_OBJECT_SRC_TRACKER_UKF_TRACKER_H_ */
