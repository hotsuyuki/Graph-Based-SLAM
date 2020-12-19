#include "PoseOptimizer_GradientDescent.h"

#include <chrono>
#include <iostream>


namespace sample_slam {

double PoseOptimizer_GradientDescent::OptimizePose(const Pose2D& pre_optimize_pose, Pose2D& post_optimize_pose) {
  double x = pre_optimize_pose.x;
  double y = pre_optimize_pose.y;
  double yaw = pre_optimize_pose.yaw;

  double min_match_cost = __DBL_MAX__;
  double min_x = x;
  double min_y = y;
  double min_yaw = yaw;

  double delta_match_cost = __DBL_MAX__;
  double prev_match_cost = __DBL_MAX__;
  int i = 0;

  while (delta_match_cost_threshold_ < delta_match_cost) {
    double partial_derivative_x = cost_function_ptr_->CalcPartialDerivativeX(x, delta_distance_, y, yaw);
    double partial_derivative_y = cost_function_ptr_->CalcPartialDerivativeY(x, y, delta_distance_, yaw);
    double partial_derivative_yaw = cost_function_ptr_->CalcPartialDerivativeYaw(x, y, yaw, delta_angle_);

    x += -step_rate_ * partial_derivative_x;
    y += -step_rate_ * partial_derivative_y;
    yaw += -step_rate_ * partial_derivative_yaw;

    double curr_match_cost = cost_function_ptr_->CalcMatchCost(x, y, yaw);

    if (curr_match_cost < min_match_cost) {
      min_match_cost = curr_match_cost;
      min_x = x;
      min_y = y;
      min_yaw = yaw;
    }

    delta_match_cost = std::fabs(prev_match_cost - curr_match_cost);
    prev_match_cost = curr_match_cost;
    ++i;
  }

  post_optimize_pose.x = min_x;
  post_optimize_pose.y = min_y;
  post_optimize_pose.yaw = min_yaw;
  post_optimize_pose.CalcRotationMatrix();

  return min_match_cost;
}

}  // namespace sample_slam