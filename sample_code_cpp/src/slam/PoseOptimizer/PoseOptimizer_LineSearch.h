#ifndef SLAM_POSEOPTIMIZER_LINESEARCH_H_
#define SLAM_POSEOPTIMIZER_LINESEARCH_H_


// slam/
#include "PoseOptimizer.h"


namespace sample_slam {

class PoseOptimizer_LineSearch : public PoseOptimizer {
 public:
  PoseOptimizer_LineSearch() : search_range_(2.0), max_iteration_(100) {}

  ~PoseOptimizer_LineSearch() {}

  double OptimizePose(const Pose2D& pre_optimize_pose, Pose2D& post_optimize_pose) override;


 private:
  void LineSearchBrent(Pose2D& search_pose, Pose2D& search_direction) const;

  double search_range_;
  int max_iteration_;
};

}  // namespace sample_slam


#endif  // SLAM_POSEOPTIMIZER_LINESEARCH_H_