#ifndef SLAM_COVARIANCECALCULATOR_H_
#define SLAM_COVARIANCECALCULATOR_H_


#include <vector>

#include <Eigen/Core>

// slam/
#include "CostFunction.h"
// utility/
#include "LaserPoint2D.h"
#include "MyUtility.h"
#include "Pose2D.h"


namespace sample_slam {

class CovarianceCalculator {
 public:
  CovarianceCalculator()
    : delta_distance_(0.00001),
      delta_angle_(0.00001),
      min_v_t_(0.02),
      min_w_t_(0.05),
      odometry_covariance_coefficient_(0.001, 0.005, 0.05),
      odometry_covariance_scale_(1.0),
      scan_match_covariance_scale_(0.1) {}

  ~CovarianceCalculator() {}

  static void RotateRelative2Absolute(const Pose2D& base_pose, const Eigen::Matrix3f& covariance_relative,
                                      Eigen::Matrix3f& covariance);

  static void RotateAbsolute2Relative(const Pose2D& base_pose, const Eigen::Matrix3f& covariance,
                                      Eigen::Matrix3f& covariance_relative);

  void CalcOdometryCovarianceRelative(const Pose2D& odometry_pose_relative, double delta_time,
                                      Eigen::Matrix3f& odometry_covariance_relative) const;

  void CalcScanMatchCovarianceICP(const Pose2D& scan_match_pose,
                                  const std::vector<LaserPoint2D>& associated_reference_laser_points,
                                  const std::vector<LaserPoint2D>& associated_current_laser_points,
                                  CostFunction* cost_function_ptr,
                                  Eigen::Matrix3f& scan_match_covariance);
  
  /*
  void CalcScanMatchCovarianceNDT(const Pose2D& scan_match_pose,
                                  const std::vector<LaserPoint2D>& associated_reference_laser_points,
                                  const std::vector<LaserPoint2D>& associated_current_laser_points,
                                  CostFunction* cost_function_ptr,
                                  Eigen::Matrix3f& scan_match_covariance);
  */


 private:
  double CalcErrorDistance(double x, double y, double yaw,
                           const LaserPoint2D& global_associated_reference_laser_point,
                           const LaserPoint2D& local_associated_current_laser_point,
                           CostFunction* cost_function_ptr);

  double delta_distance_;  // [m]
  double delta_angle_;  // [rad]
  double min_v_t_;  // [m/s]
  double min_w_t_;  // [rad/s]
  Pose2D odometry_covariance_coefficient_;
  double odometry_covariance_scale_;
  double scan_match_covariance_scale_;
};

}  // namespace sample_slam


#endif  // SLAM_COVARIANCECALCULATOR_H_