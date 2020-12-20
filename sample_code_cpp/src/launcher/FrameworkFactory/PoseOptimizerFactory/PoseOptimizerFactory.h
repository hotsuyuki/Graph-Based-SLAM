#ifndef LAUNCHER_POSEOPTIMIZER_H_
#define LAUNCHER_POSEOPTIMIZER_H_


#include <iostream>
#include <map>
#include <string>

#include "PoseOptimizer.h"
#include "PoseOptimizer_GradientDescent.h"
#include "PoseOptimizer_LineSearch.h"


namespace sample_slam {

enum class PoseOptimizerType {
  GRADIENT_DESCENT = 0,
  LINE_SEARCH = 1,
};


class PoseOptimizerFactory {
 public:
  PoseOptimizerFactory() {}

  ~PoseOptimizerFactory() {}

  static PoseOptimizer* ProducePoseOptimizerPtr(const std::string& chars) {
    std::map<PoseOptimizerType, std::string> pose_optimizer_type = {
      { PoseOptimizerType::GRADIENT_DESCENT, "gradient_descent" },
      { PoseOptimizerType::LINE_SEARCH, "line_search" },
    };

    if (chars == pose_optimizer_type[PoseOptimizerType::GRADIENT_DESCENT]) {
      return new PoseOptimizer_GradientDescent;
    } else if (chars == pose_optimizer_type[PoseOptimizerType::LINE_SEARCH]) {
      return new PoseOptimizer_LineSearch;
    } else {
      std::cerr << "[PoseOptimizerFactory::ProducePoseOptimizerPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_POSEOPTIMIZER_H_