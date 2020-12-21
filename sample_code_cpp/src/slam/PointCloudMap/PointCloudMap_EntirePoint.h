#ifndef SLAM_POINTCLOUDMAP_ENTIREPOINT_H_
#define SLAM_POINTCLOUDMAP_ENTIREPOINT_H_


// slam/
#include "PointCloudMap.h"


namespace sample_slam {

// Concrete class of `PointCloudMap` that stores entire laser point
class PointCloudMap_EntirePoint : public PointCloudMap {
 public:
  PointCloudMap_EntirePoint() : num_skip_laser_point_(5) {}

  ~PointCloudMap_EntirePoint() {}

  void AddPose(const Pose2D& pose) override;

  void AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) override;

  void MakeGlobalMap() override;

  void MakeLocalMap() override;

  void RemakeMap(const std::vector<Pose2D>& adjusted_poses) override;


 private:
  int num_skip_laser_point_;
};

}  // namespace sample_slam


#endif  // SLAM_POINTCLOUDMAP_ENTIREPOINT_H_