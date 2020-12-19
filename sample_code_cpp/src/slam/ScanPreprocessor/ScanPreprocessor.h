#ifndef SLAM_SCANPREPROCESSOR_H_
#define SLAM_SCANPREPROCESSOR_H_


#include <cmath>
#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "MyUtility.h"


namespace sample_slam {

class ScanPreprocessor {
 public:
  ScanPreprocessor()
    : min_interpolation_threshold_(0.05), max_interpolation_threshold_(0.25),
      cumulative_distance_(0.0),
      min_adjacent_threshold_(0.06), max_adjacent_threshold_(1.0),
      corner_angle_threshold_(DEG2RAD(45.0)) {}

  ~ScanPreprocessor() {}

  void ResampleLaserPoints(std::vector<LaserPoint2D>& laser_points);

  void CalcNormalVectors(std::vector<LaserPoint2D>& laser_points) const;


 private:
  void ResamplePoint(const LaserPoint2D& prev_laser_point, const LaserPoint2D& curr_laser_point,
                     LaserPoint2D& resampled_laser_point, bool& is_resampled, bool& is_interpolated);

  bool CalcLeftNormalVector(int curr_idx, const std::vector<LaserPoint2D>& laser_points,
                            double& left_normal_x, double& left_normal_y) const;

  bool CalcRightNormalVector(int curr_idx, const std::vector<LaserPoint2D>& laser_points,
                             double& right_normal_x, double& right_normal_y) const;

  double min_interpolation_threshold_;  // [m]
  double max_interpolation_threshold_;  // [m]
  double cumulative_distance_;  // [m]
  double min_adjacent_threshold_;  // [m]
  double max_adjacent_threshold_;  // [m]
  double corner_angle_threshold_;  // [rad]
};

}  // namespace sample_slam


#endif  // SLAM_SCANPREPROCESSOR_H_