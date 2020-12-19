#ifndef SLAM_SLAMFRONTEND_H_
#define SLAM_SLAMFRONTEND_H_


// slam/
#include "LoopDetector.h"
#include "PointCloudMap.h"
#include "PoseGraph.h"
#include "ScanMatcher.h"
#include "SlamBackend.h"
// utility/
#include "Scan2D.h"


namespace sample_slam {

class SlamFrontend {
 public:
  SlamFrontend()
    : point_cloud_map_ptr_(nullptr),
      num_skip_mapping_(10),
      num_point_threshold_(5),
      is_loop_closure_(false),
      pose_adjustment_iteration_(5) {
    loop_detector_.SetPoseGraphPtr(&pose_graph_);
    slam_backend_.SetPoseGraphPtr(&pose_graph_);
  }

  ~SlamFrontend() {}

  void SetPointCloudMapPtr(PointCloudMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  void SetIsLoopClosure(bool is_loop_closure) {
    is_loop_closure_ = is_loop_closure;
  }

  ScanMatcher* GetScanMatcherPtr() {
    return &scan_matcher_;
  }

  LoopDetector* GetLoopDetectorPtr() {
    return &loop_detector_;
  }

  SlamBackend* GetSlamBackendPtr() {
    return &slam_backend_;
  }

  void RunSlam(const Scan2D& scan, int count);

  void AddMoveArc(const Pose2D& estimated_pose, const Eigen::Matrix3f& move_covariance);

  void AddLoopArc(LoopInfo& loop_info);


 private:
  PointCloudMap* point_cloud_map_ptr_;  // Pointer to abstract class, which is customizable

  ScanMatcher scan_matcher_;
  PoseGraph pose_graph_;
  LoopDetector loop_detector_;
  SlamBackend slam_backend_;

  int num_skip_mapping_;
  int num_point_threshold_;
  bool is_loop_closure_;
  int pose_adjustment_iteration_;
};

}  // namespace sample_slam


#endif  // SLAM_SLAMFRONTEND_H_