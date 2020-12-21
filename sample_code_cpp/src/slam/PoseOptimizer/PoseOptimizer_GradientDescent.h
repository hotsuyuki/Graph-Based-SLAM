#ifndef SLAM_POSEOPTIMIZER_GRADIENTDESCENT_H_
#define SLAM_POSEOPTIMIZER_GRADIENTDESCENT_H_


// slam/
#include "PoseOptimizer.h"


namespace sample_slam {

class PoseOptimizer_GradientDescent : public PoseOptimizer {
 public:
  PoseOptimizer_GradientDescent()
    : step_rate_(0.00001) {}

  ~PoseOptimizer_GradientDescent() {}

  double OptimizePose(const Pose2D& pre_optimize_pose, Pose2D& post_optimize_pose) override;


 private:
  double step_rate_;
};

}  // namespace sample_slam


#endif  // SLAM_POSEOPTIMIZER_GRADIENTDESCENT_H_