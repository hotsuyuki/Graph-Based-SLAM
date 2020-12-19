#ifndef SLAM_POSEGRAPH_H_
#define SLAM_POSEGRAPH_H_


#include <vector>

#include <Eigen/Core>

// utility/
#include "Pose2D.h"


namespace sample_slam {

struct PoseNode {
  int id;
  Pose2D pose;
};


struct PoseArc {
  PoseNode* src_node_ptr;
  PoseNode* dst_node_ptr;
  Pose2D pose_relative;
  Eigen::Matrix3f information_relative;  // Inverse covariance matrix
};


class PoseGraph {
 public:
  PoseGraph() : max_pool_size_(100000) {
    nodes_pool_.reserve(max_pool_size_);
    arcs_pool_.reserve(max_pool_size_);
  }

  ~PoseGraph() {}

  int GetNodePtrsSize() const {
    return node_ptrs_.size();
  }

  PoseNode* GetNodePtrsBack() const {
    return node_ptrs_.back();
  }

  std::vector<PoseNode*>& GetNodePtrs() {
    return node_ptrs_;
  }

  std::vector<PoseArc*>& GetArcPtrs() {
    return arc_ptrs_;
  }

  PoseNode* AddNode(const Pose2D& pose);

  PoseArc* AddArc(int src_node_id, int dst_node_id,
                  const Pose2D& pose_relative, const Eigen::Matrix3f& covariance_relative);


 private:
  PoseNode* AllocNode();

  PoseArc* AllocArc();
  
  int max_pool_size_;
  std::vector<PoseNode> nodes_pool_;
  std::vector<PoseArc> arcs_pool_;
  std::vector<PoseNode*> node_ptrs_;
  std::vector<PoseArc*> arc_ptrs_;
};

}  // namespace sample_slam


#endif  // SLAM_POSEGRAPH_H_