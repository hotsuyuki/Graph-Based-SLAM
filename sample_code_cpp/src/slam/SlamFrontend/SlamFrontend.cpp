#include "SlamFrontend.h"

// slam/
#include "CovarianceCalculator.h"


namespace sample_slam {

void SlamFrontend::RunSlam(const Scan2D& scan, int count) {
  // Scan matching
  scan_matcher_.MatchScan(scan, count);

  // Pose graph
  const Pose2D& estimated_pose = point_cloud_map_ptr_->GetLastPose();
  if (count == 0) {
    pose_graph_.AddNode(estimated_pose);
  } else {
    AddMoveArc(estimated_pose, scan_matcher_.GetMoveCovariance());
  }

  // Mapping
  if (count % num_skip_mapping_ == 0) {
    if (count == 0) {
      point_cloud_map_ptr_->SetNumPointThreshold(1);
    } else {
      point_cloud_map_ptr_->SetNumPointThreshold(num_point_threshold_);
    }

    point_cloud_map_ptr_->MakeGlobalMap();
  }

  // Loop closure
  if (is_loop_closure_ && count % num_skip_mapping_ == 0 && num_skip_mapping_ < count) {
    LoopInfo loop_info;
    bool is_loop_detected = loop_detector_.DetectLoop(scan, estimated_pose, count,
                                                      loop_info);
    
    if (is_loop_detected) {
      AddLoopArc(loop_info);
      
      slam_backend_.AdjustPoses(pose_adjustment_iteration_);
      slam_backend_.RemakeMap();
    }
  }

  return;
}


void SlamFrontend::AddMoveArc(const Pose2D& estimated_pose, const Eigen::Matrix3f& move_covariance) {
  if (pose_graph_.GetNodePtrsSize() == 0) {
    std::cerr << "[SlamFrontend::AddMoveArc()] Error: `PoseGraph::node_ptrs` are empty \n";
    return;
  }

  PoseNode* last_node_ptr = pose_graph_.GetNodePtrsBack();
  PoseNode* curr_node_ptr = pose_graph_.AddNode(estimated_pose);

  Pose2D last_node_pose = last_node_ptr->pose;
  Pose2D move_pose_relative = estimated_pose - last_node_pose;

  Eigen::Matrix3f move_covariance_relative;
  CovarianceCalculator::RotateAbsolute2Relative(last_node_pose, move_covariance,
                                                move_covariance_relative);

  PoseArc* arc_ptr = pose_graph_.AddArc(last_node_ptr->id, curr_node_ptr->id,
                                        move_pose_relative, move_covariance_relative);

  return;
}


void SlamFrontend::AddLoopArc(LoopInfo& loop_info) {
  if (loop_info.is_arc) {
    return;
  }

  loop_info.is_arc = true;

  const std::vector<Pose2D>& map_poses = point_cloud_map_ptr_->GetPoses();
  Pose2D src_pose(map_poses[loop_info.reference_scan_id]);

  Pose2D dst_pose(loop_info.revisit_pose);
  Pose2D dst_pose_relative = dst_pose - src_pose;

  Eigen::Matrix3f scan_match_covariance_relative;
  CovarianceCalculator::RotateAbsolute2Relative(src_pose, loop_info.scan_match_covariance,
                                                scan_match_covariance_relative);

  PoseArc* arc_ptr = pose_graph_.AddArc(loop_info.reference_scan_id, loop_info.curr_scan_id,
                                        dst_pose_relative, scan_match_covariance_relative);

  return;
}

}  // namespace sample_slam