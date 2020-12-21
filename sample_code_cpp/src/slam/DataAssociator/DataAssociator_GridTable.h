#ifndef SLAM_DATAASSOCIATOR_GRIDTABLE_H_
#define SLAM_DATAASSOCIATOR_GRIDTABLE_H_


// slam/
#include "DataAssociator.h"
// utility/
#include "GridTable2D.h"


namespace sample_slam {

class DataAssociator_GridTable : public DataAssociator {
 public:
  DataAssociator_GridTable() {}

  ~DataAssociator_GridTable() {}

  void SetReferenceLaserPoints(const std::vector<LaserPoint2D>& reference_laser_points) override;

  double AssociateCorrespondingLaserPoints(const Pose2D& pre_optimize_pose) override;

 private:
  GridTable2D grid_table_;  // on global (map) coordinate
};

}  // namespace sample_slam


#endif  // SLAM_DATAASSOCIATOR_GRIDTABLE_H_