#include "ScanPreprocessor.h"

#include <iostream>


namespace sample_slam {

void ScanPreprocessor::ResampleLaserPoints(std::vector<LaserPoint2D>& laser_points) {
  if (laser_points.size() == 0) {
    return;
  }

  std::vector<LaserPoint2D> resampled_laser_points = {laser_points[0]};
  LaserPoint2D prev_laser_point(laser_points[0]);

  for (std::size_t i = 1; i < laser_points.size(); ++i) {
    LaserPoint2D curr_laser_point(laser_points[i]);

    LaserPoint2D resampled_laser_point;
    bool is_resampled;
    bool is_interpolated;
    ResamplePoint(prev_laser_point, curr_laser_point,
                  resampled_laser_point, is_resampled, is_interpolated);
    
    if (is_resampled) {
      resampled_laser_points.emplace_back(resampled_laser_point);
      cumulative_distance_ = 0.0;
      if (is_interpolated) {
        // Retry with the current laser point again
        // because a resampled point was interpolated before the current laser point
        --i;
      }
      prev_laser_point = resampled_laser_point;
    } else {
      prev_laser_point = curr_laser_point;
    }
  }

  std::cout << "[ScanPreprocessor::ResampleLaserPoints()] "
            << "laser_points.size() = " << laser_points.size() << ", "
            << "resampled_laser_points.size() = " << resampled_laser_points.size() << "\n";

  laser_points = resampled_laser_points;

  return;
}


void ScanPreprocessor::ResamplePoint(const LaserPoint2D& prev_laser_point, const LaserPoint2D& curr_laser_point,
                                     LaserPoint2D& resampled_laser_point, bool& is_resampled, bool& is_interpolated) {
  double dx = curr_laser_point.x - prev_laser_point.x;
  double dy = curr_laser_point.y - prev_laser_point.y;
  double distance = hypot(dx, dy);
  double total_distance = cumulative_distance_ + distance;

  if (total_distance < min_interpolation_threshold_) {
    cumulative_distance_ = total_distance;

    resampled_laser_point = LaserPoint2D();
    is_resampled = false;
    is_interpolated = false;
    return;
  }

  if (min_interpolation_threshold_ <= total_distance && total_distance < max_interpolation_threshold_) {
    double interpolation_ratio = (min_interpolation_threshold_ - cumulative_distance_) / distance;
    double resampled_x = prev_laser_point.x + interpolation_ratio * dx;
    double resampled_y = prev_laser_point.y + interpolation_ratio * dy;

    resampled_laser_point = LaserPoint2D(curr_laser_point.id, resampled_x, resampled_y);
    is_resampled = true;
    is_interpolated = true;
    return;
  }

  if (max_interpolation_threshold_ <= total_distance) {
    resampled_laser_point = curr_laser_point;
    is_resampled = true;
    is_interpolated = false;
    return;
  }
}


void ScanPreprocessor::CalcNormalVectors(std::vector<LaserPoint2D>& laser_points) const {
  for (std::size_t i = 0; i < laser_points.size(); ++i) {
    double left_normal_x, left_normal_y;
    bool is_left_normal = CalcLeftNormalVector(i, laser_points, left_normal_x, left_normal_y);

    double right_normal_x, right_normal_y;
    bool is_right_normal = CalcRightNormalVector(i, laser_points, right_normal_x, right_normal_y);

    PointType point_type;
    double normal_x, normal_y;

    if (is_left_normal && is_right_normal) {
      double inner_product = left_normal_x*right_normal_x + left_normal_y*right_normal_y;
      if (cos(corner_angle_threshold_) <= fabs(inner_product)) {
        point_type = PointType::LINE;
      } else {
        point_type = PointType::CORNER;
      }
      normal_x = left_normal_x + right_normal_x;
      normal_y = left_normal_y + right_normal_y;
      normal_x /= hypot(normal_x, normal_y);
      normal_y /= hypot(normal_x, normal_y);
    } else if (is_left_normal && !is_right_normal) {
      point_type = PointType::LINE;
      normal_x = left_normal_x;
      normal_y = left_normal_y;
    } else if (!is_left_normal && is_right_normal) {
      point_type = PointType::LINE;
      normal_x = right_normal_x;
      normal_y = right_normal_y;
    } else if (!is_left_normal && !is_right_normal) {
      point_type = PointType::ISOLATE;
      normal_x = 0.0;
      normal_y = 0.0;
    }

    laser_points[i].point_type = point_type;
    laser_points[i].normal_x = normal_x;
    laser_points[i].normal_y = normal_y;
  }

  return;
}


bool ScanPreprocessor::CalcLeftNormalVector(int curr_idx, const std::vector<LaserPoint2D>& laser_points,
                                            double& left_normal_x, double& left_normal_y) const {
  for (std::size_t i = curr_idx - 1; 0 <= i && i < laser_points.size(); --i) {
    double dx = laser_points[i].x - laser_points[curr_idx].x;
    double dy = laser_points[i].y - laser_points[curr_idx].y;
    double distance = hypot(dx, dy);
    if (distance < min_adjacent_threshold_) {
      continue;
    }

    if (min_adjacent_threshold_ <= distance && distance < max_adjacent_threshold_) {
      left_normal_x = dy / distance;
      left_normal_y = -dx / distance;
      return true;
    }

    if (max_adjacent_threshold_ <= distance ) {
      left_normal_x = 0.0;
      left_normal_y = 0.0;
      return false;
    }
  }

  left_normal_x = 0.0;
  left_normal_y = 0.0;
  return false;
}
                                          

bool ScanPreprocessor::CalcRightNormalVector(int curr_idx, const std::vector<LaserPoint2D>& laser_points,
                                             double& right_normal_x, double& right_normal_y) const {
  for (std::size_t i = curr_idx + 1; 0 <= i && i < laser_points.size(); ++i) {
    double dx = laser_points[i].x - laser_points[curr_idx].x;
    double dy = laser_points[i].y - laser_points[curr_idx].y;
    double distance = hypot(dx, dy);
    if (distance < min_adjacent_threshold_) {
      continue;
    }

    if (min_adjacent_threshold_ <= distance && distance < max_adjacent_threshold_) {
      right_normal_x = -dy / distance;
      right_normal_y = dx / distance;
      return true;
    }

    if (max_adjacent_threshold_ <= distance ) {
      right_normal_x = 0.0;
      right_normal_y = 0.0;
      return false;
    }
  }

  right_normal_x = 0.0;
  right_normal_y = 0.0;
  return false;
}

}  // namespace sample_slam