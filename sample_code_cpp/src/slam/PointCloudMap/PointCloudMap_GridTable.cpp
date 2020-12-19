#include "PointCloudMap_GridTable.h"

#include <iostream>


namespace sample_slam {

void PointCloudMap_GridTable::AddPose(const Pose2D& pose) {
  poses_.emplace_back(pose);

  return;
}


void PointCloudMap_GridTable::AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) {
  for(std::size_t i = 0; i < laser_points.size(); ++i) {
    all_laser_points_.emplace_back(laser_points[i]);
  }

  return;
}


void PointCloudMap_GridTable::MakeGlobalMap() {
  global_map_.clear();
  SubsampleMapPoints(global_map_);

  std::cout << "[PointCloudMap_GridTable::MakeGlobalMap()] "
            << "global_map_.size() = " << global_map_.size() << "\n";
}


void PointCloudMap_GridTable::MakeLocalMap() {
  local_map_ = global_map_;

  std::cout << "[PointCloudMap_GridTable::MakeLocalMap()] "
            << "local_map_.size() = " << local_map_.size() << "\n";
}


void PointCloudMap_GridTable::RemakeMap(const std::vector<Pose2D>& adjusted_poses) {
  // Dummy
}


void PointCloudMap_GridTable::SubsampleMapPoints(std::vector<LaserPoint2D>& map) {
  // auto start_clear = std::chrono::high_resolution_clock::now();
  grid_table_.clear();
  // auto end_clear = std::chrono::high_resolution_clock::now();
  // auto duration_clear = std::chrono::duration_cast<std::chrono::milliseconds>(end_clear - start_clear);
  // std::cout << "duration_clear.count() = " << duration_clear.count() << " [ms] \n";

  // auto start_AddLaserPoints = std::chrono::high_resolution_clock::now();
  for (std::size_t i = 0; i < all_laser_points_.size(); ++i) {
    grid_table_.AddLaserPoint(all_laser_points_[i]);
  }
  // auto end_AddLaserPoints = std::chrono::high_resolution_clock::now();
  // auto duration_AddLaserPoints = std::chrono::duration_cast<std::chrono::milliseconds>(end_AddLaserPoints - start_AddLaserPoints);
  // std::cout << "duration_AddLaserPoints.count() = " << duration_AddLaserPoints.count() << " [ms] \n";

  // auto start_MakeCellPoints = std::chrono::high_resolution_clock::now();
  grid_table_.MakeCellPoints(num_point_threshold_, map);
  // auto end_MakeCellPoints = std::chrono::high_resolution_clock::now();
  // auto duration_MakeCellPoints = std::chrono::duration_cast<std::chrono::milliseconds>(end_MakeCellPoints - start_MakeCellPoints);
  // std::cout << "duration_MakeCellPoints.count() = " << duration_MakeCellPoints.count() << " [ms] \n";

  std::cout << "[PointCloudMap_GridTable::SubsampleMapPoints()] "
            << "all_laser_points_.size() = " << all_laser_points_.size()
            << ", map.size() = " << map.size() << "\n";

  return;
}

}  // namespace sample_slam