#ifndef SLAM_POSEOPTIMIZER_H_
#define SLAM_POSEOPTIMIZER_H_


#include <cmath>
#include <vector>

// slam/
#include "CostFunction.h"
// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

class PoseOptimizer {
 public:
  PoseOptimizer()
    : cost_function_ptr_(nullptr),
      delta_match_cost_threshold_(0.000001),
      delta_distance_(0.00001),
      delta_angle_(0.0000001) {}

  virtual ~PoseOptimizer() {}

  void SetCostFunctionPtr(CostFunction* cost_function_ptr) {
    cost_function_ptr_ = cost_function_ptr;
  }

  CostFunction* GetCostFunctionPtr() {
    return cost_function_ptr_;
  }

  void SetAssociatedLaserPoints(const std::vector<LaserPoint2D>& associated_reference_laser_points,
                                const std::vector<LaserPoint2D>& associated_current_laser_points) {
    cost_function_ptr_->SetAssociatedLaserPoints(associated_reference_laser_points,
                                                 associated_current_laser_points);
  }

  double GetMatchedPointRatio() const {
    return cost_function_ptr_->GetMatchedPointRatio();
  }

  virtual double OptimizePose(const Pose2D& pre_optimize_pose, Pose2D& post_optimize_pose) = 0;


 protected:
  CostFunction* cost_function_ptr_;  // Pointer to abstract class, which is customizable

  double delta_match_cost_threshold_;
  double delta_distance_;  // [m]
  double delta_angle_;  // [rad]
};

}  // namespace sample_slam

#endif  // SLAM_POSEOPTIMIZER_H_