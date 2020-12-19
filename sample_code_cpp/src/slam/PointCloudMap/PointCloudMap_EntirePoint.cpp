#include "PointCloudMap_EntirePoint.h"


namespace sample_slam {

void PointCloudMap_EntirePoint::AddPose(const Pose2D& pose) {
  poses_.emplace_back(pose);

  return;
}


void PointCloudMap_EntirePoint::AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) {
  for(std::size_t i = 0; i < laser_points.size(); i += num_skip_laser_point_) {
    global_map_.emplace_back(laser_points[i]);
  }
  
  return;
}


void PointCloudMap_EntirePoint::MakeGlobalMap() {
  // Dummy
}


void PointCloudMap_EntirePoint::MakeLocalMap() {
  // Dummy
}


void PointCloudMap_EntirePoint::RemakeMap(const std::vector<Pose2D>& adjusted_poses) {
  // Dummy
}

}  // namespace sample_slam