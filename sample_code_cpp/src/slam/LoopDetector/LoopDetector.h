#ifndef SLAM_LOOPDETECTOR_H_
#define SLAM_LOOPDETECTOR_H_


#include <vector>

#include <Eigen/Core>

// slam/
#include "CostFunction_PerpendicularDistance.h"
#include "DataAssociator_GridTable.h"
#include "PointCloudMap_SubMap.h"
#include "PoseEstimator.h"
#include "PoseGraph.h"
// utility/
#include "MyUtility.h"
#include "Pose2D.h"
#include "Scan2D.h"


namespace sample_slam {

struct LoopInfo {
  bool is_arc;
  int reference_scan_id;
  int curr_scan_id;
  Pose2D revisit_pose;
  Eigen::Matrix3f scan_match_covariance;
};


class LoopDetector {
 public:
  LoopDetector()
    : min_distance_threshold_(4.0),
      cumulative_distance_threshold_(10.0),
      best_match_cost_threshold_(0.2),
      revisit_max_distance_(1.0),
      revisit_delta_distance_(0.2),
      revisit_max_angle_(DEG2RAD(45.0)),
      revisit_delta_angle_(DEG2RAD(2.0)),
      num_used_point_threshold_(50),
      association_ratio_threshold_(0.9),
      matched_point_ratio_threshold_(0.8) {}

  ~LoopDetector() {}

  void SetPointCloudMapPtr(PointCloudMap_SubMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  void SetDataAssociatorPtr(DataAssociator_GridTable* data_associator_ptr) {
    data_associator_ptr_ = data_associator_ptr;
  }

  void SetCostFunctionPtr(CostFunction_PerpendicularDistance* cost_function_ptr) {
    cost_function_ptr_ = cost_function_ptr;
  }

  void SetPoseEstimatorPtr(PoseEstimator* pose_estimator_ptr) {
    pose_estimator_ptr_ = pose_estimator_ptr;
  }

  void SetPoseGraphPtr(PoseGraph* pose_graph_ptr) {
    pose_graph_ptr_ = pose_graph_ptr;
  }

  bool DetectLoop(const Scan2D& curr_scan, const Pose2D& estimated_pose, int count,
                  LoopInfo& loop_info);


 private:
  bool SearchRevisitPose(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& estimated_pose,
                         Pose2D& revisit_pose);

  PoseEstimator* pose_estimator_ptr_;  // Pointer to abstract class, which is customizable

  PointCloudMap_SubMap* point_cloud_map_ptr_;
  DataAssociator_GridTable* data_associator_ptr_;
  CostFunction_PerpendicularDistance* cost_function_ptr_;
  PoseGraph* pose_graph_ptr_;

  double min_distance_threshold_;  // [m]
  double cumulative_distance_threshold_;  // [m]
  double best_match_cost_threshold_;
  double revisit_max_distance_;  // [m]
  double revisit_delta_distance_;  // [m]
  double revisit_max_angle_;  // [rad]
  double revisit_delta_angle_;  // [rad]
  int num_used_point_threshold_;
  double association_ratio_threshold_;
  double matched_point_ratio_threshold_;
};

}  // namespace sample_slam


#endif  // SLAM_LOOPDETECTOR_H_