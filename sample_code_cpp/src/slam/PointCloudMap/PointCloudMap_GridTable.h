#ifndef SLAM_POINTCLOUDMAP_GRIDTABLE_H_
#define SLAM_POINTCLOUDMAP_GRIDTABLE_H_


#include <vector>

// slam/
#include "PointCloudMap.h"
// utility/
#include "GridTable2D.h"
#include "LaserPoint2D.h"


namespace sample_slam {

// Concrete class of `PointCloudMap` that stores laser points in a grid table
class PointCloudMap_GridTable : public PointCloudMap {
 public:
  PointCloudMap_GridTable() {
    all_laser_points_.reserve(max_num_point_);
  }

  virtual ~PointCloudMap_GridTable() {}

  void AddPose(const Pose2D& pose) override;

  void AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) override;

  void MakeGlobalMap() override;

  void MakeLocalMap() override;

  void RemakeMap(const std::vector<Pose2D>& adjusted_poses) override;


 private:
  void SubsampleMapPoints(std::vector<LaserPoint2D>& map);

  std::vector<LaserPoint2D> all_laser_points_;
  GridTable2D grid_table_;
};

}  // namespace sample_slam


#endif  // SLAM_POINTCLOUDMAP_GRIDTABLE_H_