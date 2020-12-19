#ifndef SLAM_POSEESTIMATOR_H_
#define SLAM_POSEESTIMATOR_H_


#include <cmath>
#include <iostream>
#include <string>

#include <Eigen/Core>

// slam/
#include "CovarianceCalculator.h"
#include "DataAssociator.h"
#include "PoseOptimizer.h"
// utility/
#include "MyUtility.h"
#include "Pose2D.h"
#include "Scan2D.h"


namespace sample_slam {

class PoseEstimator {
 public:
  PoseEstimator()
    : data_associator_ptr_(nullptr),
      pose_optimizer_ptr_(nullptr),
      min_match_cost_(__DBL_MAX__),
      delta_match_cost_threshold_(0.000001),
      num_iteration_threshold_(100),
      delta_time_(0.1) {}

  virtual ~PoseEstimator() {}

  void SetDataAssociatorPtr(DataAssociator* data_associator_ptr) {
    data_associator_ptr_ = data_associator_ptr;
  }

  void SetPoseOptimizerPtr(PoseOptimizer* pose_optimizer_ptr) {
    pose_optimizer_ptr_ = pose_optimizer_ptr;
  }

  PoseOptimizer* GetPoseOptimizerPtr() {
    return pose_optimizer_ptr_;
  }

  void SetReferenceScan(const Scan2D& reference_scan) {
    data_associator_ptr_->SetReferenceLaserPoints(reference_scan.laser_points);
  }

  void SetCurrentScan(const Scan2D& curr_scan) {
    data_associator_ptr_->SetCurrentLaserPoints(curr_scan.laser_points);
  }

  const Pose2D& GetEstimatedPose() const {
    return estimated_pose_;
  }

  double GetMinMatchCost() const {
    return min_match_cost_;
  }

  int GetNumUsedPoint() const {
    return num_used_point_;
  }

  double GetMatchedPointRatio() const {
    return matched_point_ratio_;
  }

  void OdometryFusion(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& scan_match_pose,
                      const Pose2D& last_pose, const Pose2D& odometry_pose_relative, const Pose2D& odometry_pose,
                      Pose2D& fused_pose, Eigen::Matrix3f& fused_covariance) {
    Eigen::Vector3f scan_match_mean(scan_match_pose.x, scan_match_pose.y, scan_match_pose.yaw);

    Eigen::Matrix3f scan_match_covariance;
    CalcScanMatchCovariance(reference_scan, curr_scan, scan_match_pose,
                            scan_match_covariance);

    Eigen::Vector3f odometry_mean(odometry_pose.x, odometry_pose.y, odometry_pose.yaw);

    Eigen::Matrix3f odometry_covariance;
    CalcOdometryCovariance(last_pose, odometry_pose_relative,
                           odometry_covariance);
    /*
    CalcOdometryCovariance(scan_match_pose, odometry_pose_relative,
                           odometry_covariance);
    */
    
    Eigen::Vector3f fused_mean;
    FuseNormalDistributions(scan_match_mean, scan_match_covariance,
                            odometry_mean, odometry_covariance,
                            fused_mean, fused_covariance);

    fused_pose.x = fused_mean(0);
    fused_pose.y = fused_mean(1);
    fused_pose.yaw = fused_mean(2);
    fused_pose.CalcRotationMatrix();

    std::cout << "[PoseEstimator::OdometryFusion()] \n"
              << "odometry_pose.x = " << odometry_pose.x << ", odometry_pose.y = " << odometry_pose.y
              << ", odometry_pose.yaw = " << odometry_pose.yaw << "\n"
              << "scan_match_pose.x = " << scan_match_pose.x << ", scan_match_pose.y = " << scan_match_pose.y
              << ", scan_match_pose.yaw = " << scan_match_pose.yaw << "\n"
              << "fused_pose.x = " << fused_pose.x << ", fused_pose.y = " << fused_pose.y
              << ", fused_pose.yaw = " << fused_pose.yaw << "\n";

    return;
  }
 
  void CalcOdometryCovariance(const Pose2D& last_pose, const Pose2D& odometry_pose_relative,
                              Eigen::Matrix3f& odometry_covariance) {
    Eigen::Matrix3f odometry_covariance_relative;
    covariance_calculator_.CalcOdometryCovarianceRelative(odometry_pose_relative, delta_time_,
                                                        odometry_covariance_relative);

    CovarianceCalculator::RotateRelative2Absolute(last_pose, odometry_covariance_relative,
                                                  odometry_covariance);

    return;
  }

  virtual void CalcScanMatchCovariance(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& scan_match_pose,
                                       Eigen::Matrix3f& scan_match_covariance) = 0;
 
  virtual void EstimatePose(const Pose2D& init_pose) = 0;


 protected:
  void FuseNormalDistributions(const Eigen::Vector3f& mean1, const Eigen::Matrix3f& covariance1,
                               const Eigen::Vector3f& mean2, const Eigen::Matrix3f& covariance2,
                               Eigen::Vector3f& fused_mean, Eigen::Matrix3f& fused_covariance) {
    Eigen::Matrix3f inverse_covariance1;
    MyUtility::InverseMatrixSVD(covariance1, inverse_covariance1);

    Eigen::Matrix3f inverse_covariance2;
    MyUtility::InverseMatrixSVD(covariance2, inverse_covariance2);

    // S' = (S_1^{-1} + S_2^{-1})^{-1}
    Eigen::Matrix3f inverse_fused_covariance = inverse_covariance1 + inverse_covariance2;
    MyUtility::InverseMatrixSVD(inverse_fused_covariance, fused_covariance);

    // Angle correction
    Eigen::Vector3f mean1_correct = mean1;
    double delta_angle =  mean2(2) - mean1(2);  // [rad]
    if (delta_angle < -M_PI) {
      mean1_correct(2) -= 2.0*M_PI;
    } else if (M_PI < delta_angle) {
      mean1_correct(2) += 2.0*M_PI;
    }

    // x' = S'*(S_1^{-1}*x_1 + S_2^{-1}*x_2)
    fused_mean = fused_covariance*(inverse_covariance1*mean1_correct + inverse_covariance2*mean2);
    fused_mean(2) = MyUtility::CastRadian(fused_mean(2));

    return;
  }

  DataAssociator* data_associator_ptr_;  // Pointer to abstract class, which is customizable
  PoseOptimizer* pose_optimizer_ptr_;  // Pointer to abstract class, which is customizable

  double min_match_cost_;
  double delta_match_cost_threshold_;
  int num_iteration_threshold_;
  double delta_time_;

  int num_used_point_;
  double matched_point_ratio_;
  Pose2D estimated_pose_;
  CovarianceCalculator covariance_calculator_;
};

}  // namespace sample_slam


#endif  // SLAM_POSEESTIMATOR_H_