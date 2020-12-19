#include "PoseOptimizer_LineSearch.h"

#include <boost/math/tools/minima.hpp>

// utility/
#include "MyUtility.h"


namespace sample_slam {

double PoseOptimizer_LineSearch::OptimizePose(const Pose2D& pre_optimize_pose, Pose2D& post_optimize_pose) {
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

    x += partial_derivative_x;
    y += partial_derivative_y;
    yaw += partial_derivative_yaw;

    Pose2D search_pose(x, y, yaw);
    Pose2D search_direction(partial_derivative_x, partial_derivative_y, partial_derivative_yaw);
    LineSearchBrent(search_pose, search_direction);

    x = search_pose.x;
    y = search_pose.y;
    yaw = search_pose.yaw;

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


void PoseOptimizer_LineSearch::LineSearchBrent(Pose2D& search_pose, Pose2D& search_direction) const {
  boost::uintmax_t max_iteration = max_iteration_;

  std::pair<double, double> result = boost::math::tools::brent_find_minima(
    [this, &search_pose, &search_direction](double step_rate_tmp) {
      double x = search_pose.x + step_rate_tmp * search_direction.x;
      double y = search_pose.y + step_rate_tmp * search_direction.y;
      double yaw = MyUtility::AddRadian(search_pose.yaw, step_rate_tmp * search_direction.yaw);
      double match_cost = cost_function_ptr_->CalcMatchCost(x, y, yaw);
      return match_cost;
    },
    -search_range_, search_range_, std::numeric_limits<double>::digits, max_iteration
  );

  double step_rate = result.first;
  // double minima = result.second;

  search_pose.x += step_rate * search_direction.x;
  search_pose.y += step_rate * search_direction.y;
  search_pose.yaw = MyUtility::AddRadian(search_pose.yaw, step_rate * search_direction.yaw);

  return;
}

}  // namespace sample_slam