#ifndef SLAM_DATAASSOCIATOR_LINEARSEARCH_H_
#define SLAM_DATAASSOCIATOR_LINEARSEARCH_H_


// slam/
#include "DataAssociator.h"


namespace sample_slam {

class DataAssociator_LinearSearch : public DataAssociator {
 public:
  DataAssociator_LinearSearch()
    : distance_threshold_(0.2) {}

  virtual ~DataAssociator_LinearSearch() {}

  void SetReferenceLaserPoints(const std::vector<LaserPoint2D>& reference_laser_points) override;

  double AssociateCorrespondingLaserPoints(const Pose2D& pre_optimize_pose) override;

 private:
  std::vector<LaserPoint2D> reference_laser_points_;  // on global (map) coordinate
  double distance_threshold_;
};

}  // namespace sample_slam


#endif  // SLAM_DATAASSOCIATOR_LINEARSEARCH_H_