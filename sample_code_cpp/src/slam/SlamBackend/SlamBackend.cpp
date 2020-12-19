#include "SlamBackend.h"

#include <fstream> // It is needed to use p2o.h

#include "p2o.h"


namespace sample_slam {

void SlamBackend::AdjustPoses(int pose_adjustment_iteration) {
  const std::vector<PoseNode*>& node_ptrs = pose_graph_ptr_->GetNodePtrs();
  std::vector<p2o::Pose2D> p2o_poses;
  for (std::size_t i = 0; i < node_ptrs.size(); ++i) {
    const Pose2D& node_pose = node_ptrs[i]->pose;
    p2o_poses.emplace_back(p2o::Pose2D(node_pose.x, node_pose.y, node_pose.yaw));
  }

  const std::vector<PoseArc*>& arc_ptrs = pose_graph_ptr_->GetArcPtrs();
  p2o::Con2DVec p2o_cons;
  for (std::size_t i = 0; i < arc_ptrs.size(); ++i) {
    p2o::Con2D p2o_con;
    p2o_con.id1 = arc_ptrs[i]->src_node_ptr->id;
    p2o_con.id2 = arc_ptrs[i]->dst_node_ptr->id;

    const Pose2D& arc_pose_relative = arc_ptrs[i]->pose_relative;
    p2o_con.t = p2o::Pose2D(arc_pose_relative.x, arc_pose_relative.y, arc_pose_relative.yaw);

    const Eigen::Matrix3f& arc_information_relative = arc_ptrs[i]->information_relative;
    int dimension = 3;
    for (std::size_t h = 0; h < dimension; ++h) {
      for (std::size_t w = 0; w < dimension; ++w) {
        p2o_con.info(h, w) = arc_information_relative(h, w);
      }
    }
    
    p2o_cons.emplace_back(p2o_con);
  }

  p2o::Optimizer2D p2o_optimizer;
  std::vector<p2o::Pose2D> p2o_adjusted_poses = p2o_optimizer.optimizePath(p2o_poses,
                                                                           p2o_cons,
                                                                           pose_adjustment_iteration);
  
  adjusted_poses_.clear();
  for (std::size_t i = 0; i < p2o_adjusted_poses.size(); ++i) {
    Pose2D adjusted_pose(p2o_adjusted_poses[i].x, p2o_adjusted_poses[i].y, p2o_adjusted_poses[i].th);
    adjusted_poses_.emplace_back(adjusted_pose);
  }
  
  return;
}


void SlamBackend::RemakeMap() {
  std::vector<PoseNode*>& node_ptrs = pose_graph_ptr_->GetNodePtrs();
  for (std::size_t i = 0; i < adjusted_poses_.size(); ++i) {
    node_ptrs[i]->pose = adjusted_poses_[i];
  }

  point_cloud_map_ptr_->RemakeMap(adjusted_poses_);

  return;
}

}  // namespace sample_slam