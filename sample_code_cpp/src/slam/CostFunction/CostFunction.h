#ifndef SLAM_COSTFUNCTION_H_
#define SLAM_COSTFUNCTION_H_


#include <cmath>
#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

class CostFunction {
 public:
  CostFunction()
    : error_distance_threshold_(0.2),
      match_cost_scale_(100.0) {}

  virtual ~CostFunction() {}

  void SetAssociatedLaserPoints(const std::vector<LaserPoint2D>& associated_reference_laser_points,
                                const std::vector<LaserPoint2D>& associated_current_laser_points) {
    associated_reference_laser_points_ = associated_reference_laser_points;
    associated_current_laser_points_ = associated_current_laser_points;
  }

  double GetMatchedPointRatio() const {
    return matched_point_ratio_;
  }

  double CalcPartialDerivativeX(double x, double delta_distance, double y, double yaw) {
    return (CalcMatchCost(x + delta_distance, y, yaw) - CalcMatchCost(x, y, yaw)) / delta_distance;
  }

  double CalcPartialDerivativeY(double x, double y, double delta_distance, double yaw) {
    return (CalcMatchCost(x, y + delta_distance, yaw) - CalcMatchCost(x, y, yaw)) / delta_distance;
  }

  double CalcPartialDerivativeYaw(double x, double y, double yaw, double delta_angle) {
    return (CalcMatchCost(x, y, yaw + delta_angle) - CalcMatchCost(x, y, yaw)) / delta_angle;
  }

  double CalcMatchCost(double x, double y, double yaw) {
    int num_matched_point = 0;
    int num_all_point = 0;
    double sum_square_error_distance = 0.0;  // [m^2]

    for (std::size_t i = 0; i < associated_current_laser_points_.size(); ++i) {
      const LaserPoint2D& global_associated_reference_laser_point_i = associated_reference_laser_points_[i];  // on global (map) coordinate
      const LaserPoint2D& local_associated_current_laser_point_i = associated_current_laser_points_[i];  // on local (robot) coordinate

      if (global_associated_reference_laser_point_i.point_type != PointType::LINE) {
        continue;
      }

      Pose2D pose_tmp(x, y, yaw);
      LaserPoint2D global_associated_current_laser_point_i;
      pose_tmp.LocalCoord2GlobalCoord(local_associated_current_laser_point_i,
                                      global_associated_current_laser_point_i);

      double error_distance = CalcErrorDistance(global_associated_reference_laser_point_i,
                                                global_associated_current_laser_point_i);

      if (error_distance < error_distance_threshold_) {
        ++num_matched_point;
      }

      ++num_all_point;
      sum_square_error_distance += error_distance * error_distance;  // [m^2]
    }

    double num_all_point_double = static_cast<double>(num_all_point) + __DBL_EPSILON__;

    double mean_square_error_distance = sum_square_error_distance / num_all_point_double;
    matched_point_ratio_ = static_cast<double>(num_matched_point) / num_all_point_double;

    // Scale up the MSE in order to avoid being too small value
    double match_cost = match_cost_scale_ * mean_square_error_distance;

    return match_cost;
  }
  
  virtual double CalcErrorDistance(const LaserPoint2D& associated_reference_laser_point,
                                   const LaserPoint2D& associated_current_laser_point) = 0;


 protected:
  double error_distance_threshold_;
  double match_cost_scale_;
  double matched_point_ratio_;

  std::vector<LaserPoint2D> associated_reference_laser_points_;  // on global (map) coordinate
  std::vector<LaserPoint2D> associated_current_laser_points_;  // on local (robot) coordinate
};

}  // namespace sample_slam


#endif  // SLAM_COSTFUNCTION_H_