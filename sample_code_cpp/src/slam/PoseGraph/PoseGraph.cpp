#include "PoseGraph.h"

#include <iostream>


namespace sample_slam {

PoseNode* PoseGraph::AddNode(const Pose2D& pose) {
  PoseNode* node_ptr = AllocNode();

  node_ptr->id = static_cast<int>(node_ptrs_.size());
  node_ptr->pose = pose;
  node_ptrs_.emplace_back(node_ptr);

  return node_ptr;
}


PoseArc* PoseGraph::AddArc(int src_node_id, int dst_node_id,
                            const Pose2D& pose_relative, const Eigen::Matrix3f& covariance_relative) {
  // Omega = Sigma^{-1}
  Eigen::Matrix3f information_relative;
  MyUtility::InverseMatrixSVD(covariance_relative, information_relative);

  PoseArc* arc_ptr = AllocArc();
  arc_ptr->src_node_ptr = node_ptrs_[src_node_id];
  arc_ptr->dst_node_ptr = node_ptrs_[dst_node_id];
  arc_ptr->pose_relative = pose_relative;
  arc_ptr->information_relative = information_relative;

  arc_ptrs_.emplace_back(arc_ptr);

  return arc_ptr;
}


PoseNode* PoseGraph::AllocNode() {
  if (max_pool_size_ <= nodes_pool_.size()) {
    std::cerr << "[PoseGraph::AllocNode()] Error: nodes_pool_ exceeds its capacity \n";
    return nullptr;
  }

  PoseNode node;
  nodes_pool_.emplace_back(node);
  PoseNode* alloced_node_ptr = &(nodes_pool_.back());

  return alloced_node_ptr;
}


PoseArc* PoseGraph::AllocArc() {
  if (max_pool_size_ <= arcs_pool_.size()) {
    std::cerr << "[PoseGraph::AllocArc()] Error: arcs_pool_ exceeds its capacity \n";
    return nullptr;
  }

  PoseArc arc;
  arcs_pool_.emplace_back(arc);
  PoseArc* alloced_arc_ptr = &(arcs_pool_.back());

  return alloced_arc_ptr;
}

}  // namespace sample_slam