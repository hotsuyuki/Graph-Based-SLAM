#include "DataAssociator_LinearSearch.h"

#include <cmath>


namespace sample_slam {

void DataAssociator_LinearSearch::SetReferenceLaserPoints(const std::vector<LaserPoint2D>& reference_laser_points) {
  reference_laser_points_ = reference_laser_points;  // on global (map) coordinate

  return;
}


double DataAssociator_LinearSearch::AssociateCorrespondingLaserPoints(const Pose2D& pre_optimize_pose) {
  associated_reference_laser_points_.clear();  // on global (map) coordinate
  associated_current_laser_points_.clear();  // on local (robot) coordinate

  for (std::size_t i = 0; i < curr_laser_points_.size(); ++i) {
    LaserPoint2D& local_curr_laser_point_i = curr_laser_points_[i];

    LaserPoint2D global_curr_laser_point;
    pre_optimize_pose.LocalCoord2GlobalCoord(local_curr_laser_point_i, global_curr_laser_point);

    double min_distance = __DBL_MAX__;
    LaserPoint2D* reference_laser_point_j_ptr = nullptr;
    
    for (std::size_t j = 0; j < reference_laser_points_.size(); ++j) {
      double dx = global_curr_laser_point.x - reference_laser_points_[j].x;
      double dy = global_curr_laser_point.y - reference_laser_points_[j].y;
      double distance = hypot(dx, dy);

      if (distance < distance_threshold_ && distance < min_distance) {
        min_distance = distance;
        reference_laser_point_j_ptr = &(reference_laser_points_[j]);
      }
    }

    if (reference_laser_point_j_ptr != nullptr) {
      associated_reference_laser_points_.emplace_back(*reference_laser_point_j_ptr);  // on global (map) coordinate
      associated_current_laser_points_.emplace_back(local_curr_laser_point_i);  // on local (robot) coordinate
    }
  }

  double association_ratio = static_cast<double>(associated_current_laser_points_.size())
                               / static_cast<double>(curr_laser_points_.size());

  return association_ratio;
}

}  // namespace sample_slam