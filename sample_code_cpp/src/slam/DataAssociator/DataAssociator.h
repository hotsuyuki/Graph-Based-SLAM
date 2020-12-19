#ifndef SLAM_DATAASSOCIATOR_H_
#define SLAM_DATAASSOCIATOR_H_


#include <cmath>
#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

class DataAssociator {
 public:
  DataAssociator() {}

  virtual ~DataAssociator() {}

  void SetCurrentLaserPoints(const std::vector<LaserPoint2D>& curr_laser_points) {
    curr_laser_points_ = curr_laser_points;  // on local (robot) coordinate
  }

  const std::vector<LaserPoint2D>& GetAssociatedReferenceLaserPoints() const {
    return associated_reference_laser_points_;
  }

  const std::vector<LaserPoint2D>& GetAssociatedCurrentLaserPoints() const {
    return associated_current_laser_points_;
  }

  virtual void SetReferenceLaserPoints(const std::vector<LaserPoint2D>& reference_laser_points) = 0;

  virtual double AssociateCorrespondingLaserPoints(const Pose2D& pre_optimize_pose) = 0;


 protected:
  std::vector<LaserPoint2D> curr_laser_points_;  // on local (robot) coordinate

  std::vector<LaserPoint2D> associated_reference_laser_points_;  // on global (map) coordinate
  std::vector<LaserPoint2D> associated_current_laser_points_;  // on local (robot) coordinate
};

}  // namespace sample_slam


#endif  // SLAM_DATAASSOCIATOR_H_