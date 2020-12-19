#include "DataAssociator_GridTable.h"


namespace sample_slam {

void DataAssociator_GridTable::SetReferenceLaserPoints(const std::vector<LaserPoint2D>& reference_laser_points) {
  grid_table_.clear();  // on global (map) coordinate
  for (std::size_t i = 0; i < reference_laser_points.size(); ++i) {
    grid_table_.AddLaserPoint(&(reference_laser_points[i]));
  }

  return;
}


double DataAssociator_GridTable::AssociateCorrespondingLaserPoints(const Pose2D& pre_optimize_pose) {
  associated_reference_laser_points_.clear();  // on global (map) coordinate
  associated_current_laser_points_.clear();  // on local (robot) coordinate

  for (std::size_t i = 0; i < curr_laser_points_.size(); ++i) {
    LaserPoint2D& local_curr_laser_point = curr_laser_points_[i];

    LaserPoint2D global_curr_laser_point;
    pre_optimize_pose.LocalCoord2GlobalCoord(local_curr_laser_point, global_curr_laser_point);

    const LaserPoint2D* reference_laser_point_ptr = grid_table_.FindNearestLaserPoint(global_curr_laser_point);
    
    if (reference_laser_point_ptr != nullptr) {
      associated_reference_laser_points_.emplace_back(*reference_laser_point_ptr);  // on global (map) coordinate
      associated_current_laser_points_.emplace_back(local_curr_laser_point);  // on local (robot) coordinate
    }
  }

  double association_ratio = static_cast<double>(associated_current_laser_points_.size())
                               / static_cast<double>(curr_laser_points_.size());

  return association_ratio;
}

}  // namespace sample_slam