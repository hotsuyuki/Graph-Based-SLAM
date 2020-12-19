#ifndef SLAM_SCANMATCHER_H_
#define SLAM_SCANMATCHER_H_


// slam/
#include "PointCloudMap.h"
#include "PoseEstimator.h"
#include "ReferenceScanMaker.h"
// utility/
#include "Pose2D.h"
#include "Scan2D.h"


namespace sample_slam {

class ScanMatcher {
 public:
  ScanMatcher()
    : point_cloud_map_ptr_(nullptr),
      reference_scan_maker_ptr_(nullptr),
      pose_estimator_ptr_(nullptr),
      is_odometry_fusion_(false),
      match_cost_threshold_(1.0), num_used_point_threshold_(50) {}

  ~ScanMatcher() {}

  void SetPointCloudMapPtr(PointCloudMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  void SetReferenceScanMakerPtr(ReferenceScanMaker* reference_scan_maker_ptr) {
    reference_scan_maker_ptr_ = reference_scan_maker_ptr;
  }

  void SetPoseEstimatorPtr(PoseEstimator* pose_estimator_ptr) {
    pose_estimator_ptr_ = pose_estimator_ptr;
  }

  void SetIsScanPreprocess(bool is_scan_preprocess) {
    is_scan_preprocess_ = is_scan_preprocess;
  }

  void SetIsOdometryFusion(bool is_odometry_fusion) {
    is_odometry_fusion_ = is_odometry_fusion;
  }

  ReferenceScanMaker* GetReferenceScanMakerPtr() {
    return reference_scan_maker_ptr_;
  }

  PoseEstimator* GetPoseEstimatorPtr() {
    return pose_estimator_ptr_;
  }

  const Eigen::Matrix3f& GetMoveCovariance() {
    return move_covariance_;
  }

  bool MatchScan(const Scan2D& scan, int count);

  void GrowMap(const Scan2D& curr_scan, const Pose2D& estimated_pose, int count);


 private:
  PointCloudMap* point_cloud_map_ptr_;  // Pointer to abstract class, which is customizable
  ReferenceScanMaker* reference_scan_maker_ptr_;  // Pointer to abstract class, which is customizable
  PoseEstimator* pose_estimator_ptr_;  // Pointer to abstract class, which is customizable
  bool is_scan_preprocess_;
  bool is_odometry_fusion_;

  double match_cost_threshold_;
  int num_used_point_threshold_;

  Scan2D prev_scan_;
  Eigen::Matrix3f move_covariance_;  // Covariance of the relative movement in one frame
};

}  // namespace sample_slam

#endif  // SLAM_SCANMATCHER_H_