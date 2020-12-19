#include "PoseEstimator_ICP.h"


namespace sample_slam {

void PoseEstimator_ICP::EstimatePose(const Pose2D& init_pose) {
  min_match_cost_ = __DBL_MAX__;

  Pose2D pre_optimize_pose = init_pose;
  double delta_match_cost = __DBL_MAX__;
  double prev_match_cost = __DBL_MAX__;
  int i = 0;

  while (delta_match_cost_threshold_ < delta_match_cost) {
    data_associator_ptr_->AssociateCorrespondingLaserPoints(pre_optimize_pose);

    pose_optimizer_ptr_->SetAssociatedLaserPoints(data_associator_ptr_->GetAssociatedReferenceLaserPoints(),
                                                  data_associator_ptr_->GetAssociatedCurrentLaserPoints());

    Pose2D post_optimize_pose;
    double curr_match_cost = pose_optimizer_ptr_->OptimizePose(pre_optimize_pose, post_optimize_pose);

    if (curr_match_cost < min_match_cost_) {
      min_match_cost_ = curr_match_cost;
      estimated_pose_ = post_optimize_pose;
    }

    pre_optimize_pose = post_optimize_pose;
    delta_match_cost = std::fabs(prev_match_cost - curr_match_cost);
    prev_match_cost = curr_match_cost;
    ++i;

    if (num_iteration_threshold_ < i) {
      break;
    }
  }

  num_used_point_ = data_associator_ptr_->GetAssociatedCurrentLaserPoints().size();
  matched_point_ratio_ = pose_optimizer_ptr_->GetMatchedPointRatio();

  return;
}


void PoseEstimator_ICP::CalcScanMatchCovariance(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& scan_match_pose,
                                                Eigen::Matrix3f& scan_match_covariance) {
  data_associator_ptr_->SetReferenceLaserPoints(reference_scan.laser_points);
  data_associator_ptr_->SetCurrentLaserPoints(curr_scan.laser_points);
  data_associator_ptr_->AssociateCorrespondingLaserPoints(scan_match_pose);

  covariance_calculator_.CalcScanMatchCovarianceICP(scan_match_pose,
                                                    data_associator_ptr_->GetAssociatedReferenceLaserPoints(),
                                                    data_associator_ptr_->GetAssociatedCurrentLaserPoints(),
                                                    pose_optimizer_ptr_->GetCostFunctionPtr(),
                                                    scan_match_covariance);

  return;
}

}  // namespace sample_slam