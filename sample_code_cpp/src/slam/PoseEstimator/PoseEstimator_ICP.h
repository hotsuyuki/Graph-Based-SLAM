#ifndef SLAM_POSEESTIMATOR_ICP_H_
#define SLAM_POSEESTIMATOR_ICP_H_


// slam/
#include "PoseEstimator.h"


namespace sample_slam {

class PoseEstimator_ICP : public PoseEstimator {
 public:
  PoseEstimator_ICP() {}

  ~PoseEstimator_ICP() {}

  void EstimatePose(const Pose2D& init_pose) override;

  void CalcScanMatchCovariance(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& scan_match_pose,
                               Eigen::Matrix3f& scan_match_covariance) override;
};

}  // namespace sample_slam


#endif  // SLAM_POSEESTIMATOR_ICP_H_