#include "CovarianceCalculator.h"

#include <cmath>


namespace sample_slam {

void CovarianceCalculator::RotateRelative2Absolute(const Pose2D& base_pose, const Eigen::Matrix3f& covariance_relative,
                                                   Eigen::Matrix3f& covariance) {
  double cos_yaw = cos(base_pose.yaw);
  double sin_yaw = sin(base_pose.yaw);

  Eigen::Matrix3f rotation_rel2abs;
  rotation_rel2abs << cos_yaw, -sin_yaw, 0.0,
                      sin_yaw,  cos_yaw, 0.0,
                          0.0,      0.0, 1.0;

  covariance = rotation_rel2abs * covariance_relative * rotation_rel2abs.transpose();

  return;
}


void CovarianceCalculator::RotateAbsolute2Relative(const Pose2D& base_pose, const Eigen::Matrix3f& covariance,
                                                   Eigen::Matrix3f& covariance_relative) {
  double cos_yaw = cos(base_pose.yaw);
  double sin_yaw = sin(base_pose.yaw);

  Eigen::Matrix3f rotation_abs2rel;
  rotation_abs2rel <<  cos_yaw, sin_yaw, 0.0,
                      -sin_yaw, cos_yaw, 0.0,
                           0.0,     0.0, 1.0;

  covariance_relative = rotation_abs2rel * covariance * rotation_abs2rel.transpose();

  return;
}


void CovarianceCalculator::CalcOdometryCovarianceRelative(const Pose2D& odometry_pose_relative, double delta_time,
                                                          Eigen::Matrix3f& odometry_covariance_relative) const {
  double v_t = hypot(odometry_pose_relative.x, odometry_pose_relative.y) / delta_time;  // [m/s]
  double w_t = odometry_pose_relative.yaw / delta_time;  // [rad/s]

  v_t = std::max(v_t, min_v_t_);
  w_t = std::max(w_t, min_w_t_);

  odometry_covariance_relative = Eigen::Matrix3f::Zero(3, 3);
  odometry_covariance_relative(0,0) = odometry_covariance_coefficient_.x * v_t * v_t;
  odometry_covariance_relative(1,1) = odometry_covariance_coefficient_.y * v_t * v_t;
  odometry_covariance_relative(2,2) = odometry_covariance_coefficient_.yaw * w_t * w_t;

  // Adjusts scale depending on the scale of scan match covariance
  odometry_covariance_relative *= odometry_covariance_scale_;

  return;
}


void CovarianceCalculator::CalcScanMatchCovarianceICP(const Pose2D& scan_match_pose,
                                                      const std::vector<LaserPoint2D>& associated_reference_laser_points,
                                                      const std::vector<LaserPoint2D>& associated_current_laser_points,
                                                      CostFunction* cost_function_ptr,
                                                      Eigen::Matrix3f& scan_match_covariance) {
  double x = scan_match_pose.x;  // [m]
  double y = scan_match_pose.y;  // [m]
  double yaw = scan_match_pose.yaw;  // [yaw]

  std::vector<float> jacobi_x;  // column vector w.r.t. x
  std::vector<float> jacobi_y;  // column vector w.r.t. y
  std::vector<float> jacobi_yaw;  // column vector w.r.t. yaw

  for (std::size_t i = 0; i < associated_current_laser_points.size(); ++i) {
    const LaserPoint2D& global_associated_reference_laser_point_i = associated_reference_laser_points[i];  // on global (map) coordinate
    const LaserPoint2D& local_associated_current_laser_point_i = associated_current_laser_points[i];  // on local (robot) coordinate

    if (global_associated_reference_laser_point_i.point_type == PointType::ISOLATE) {
      continue;
    }

    double error_distance = CalcErrorDistance(x, y, yaw,
                                              global_associated_reference_laser_point_i,
                                              local_associated_current_laser_point_i,
                                              cost_function_ptr);

    double error_distance_x = CalcErrorDistance(x + delta_distance_, y, yaw,
                                                global_associated_reference_laser_point_i,
                                                local_associated_current_laser_point_i,
                                                cost_function_ptr);

    double error_distance_y = CalcErrorDistance(x, y + delta_distance_, yaw,
                                                global_associated_reference_laser_point_i,
                                                local_associated_current_laser_point_i,
                                                cost_function_ptr);
                                                 
    double error_distance_yaw = CalcErrorDistance(x, y, yaw + delta_angle_,
                                                  global_associated_reference_laser_point_i,
                                                  local_associated_current_laser_point_i,
                                                  cost_function_ptr);

    float partial_derivative_x = static_cast<float>((error_distance_x - error_distance) / delta_distance_);
    float partial_derivative_y = static_cast<float>((error_distance_y - error_distance) / delta_distance_);
    float partial_derivative_yaw = static_cast<float>((error_distance_yaw - error_distance) / delta_angle_);

    jacobi_x.push_back(partial_derivative_x);
    jacobi_y.push_back(partial_derivative_y);
    jacobi_yaw.push_back(partial_derivative_yaw);
  }

  // H = J^T * J
  Eigen::Matrix3f hess = Eigen::Matrix3f::Zero(3, 3);
  for (std::size_t i = 0; i < jacobi_x.size(); ++i) {
    hess(0,0) += jacobi_x[i]*jacobi_x[i];
    hess(1,0) += jacobi_y[i]*jacobi_x[i];    hess(1,1) += jacobi_y[i]*jacobi_y[i];
    hess(2,0) += jacobi_yaw[i]*jacobi_x[i];  hess(2,1) += jacobi_yaw[i]*jacobi_y[i];  hess(2,2) += jacobi_yaw[i]*jacobi_yaw[i];
  }

  // H^T = H
  hess(0,1) = hess(1,0);  hess(0,2) = hess(2,0);
                          hess(1,2) = hess(2,1);

  // S = H^{-1} 
  MyUtility::InverseMatrixSVD(hess, scan_match_covariance); 

  // Adjusts scale depending on the scale of odometry covariance
  scan_match_covariance *= scan_match_covariance_scale_;

  return;
}


double CovarianceCalculator::CalcErrorDistance(double x, double y, double yaw,
                                               const LaserPoint2D& global_associated_reference_laser_point,
                                               const LaserPoint2D& local_associated_current_laser_point,
                                               CostFunction* cost_function_ptr) {
  Pose2D pose_tmp(x, y, yaw);
  LaserPoint2D global_associated_current_laser_point;
  pose_tmp.LocalCoord2GlobalCoord(local_associated_current_laser_point,
                                  global_associated_current_laser_point);

  double error_distance = cost_function_ptr->CalcErrorDistance(global_associated_reference_laser_point,
                                                               global_associated_current_laser_point);

  return error_distance;
}

}  // namespace sample_slam