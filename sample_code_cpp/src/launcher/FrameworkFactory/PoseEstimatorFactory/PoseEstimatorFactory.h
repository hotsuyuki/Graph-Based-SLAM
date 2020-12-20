#ifndef LAUNCHER_POSEESTIMATORFACTORY_H_
#define LAUNCHER_POSEESTIMATORFACTORY_H_


#include <iostream>
#include <map>
#include <string>

#include "PoseEstimator.h"
#include "PoseEstimator_ICP.h"
// #include "PoseEstimator_NDT.h"


namespace sample_slam {

enum class PoseEstimatorType {
  ICP = 0,
  // NDT = 1,
};


class PoseEstimatorFactory {
 public:
  PoseEstimatorFactory() {}

  ~PoseEstimatorFactory() {}

  static PoseEstimator* ProducePoseEstimatorPtr(const std::string& chars) {
    std::map<PoseEstimatorType, std::string> pose_estimator_type = {
      { PoseEstimatorType::ICP, "icp" },
      // { PoseEstimatorType::NDT, "ndt" },
    };

    if (chars == pose_estimator_type[PoseEstimatorType::ICP]) {
      return new PoseEstimator_ICP;
    // } else if (chars == pose_estimator_type[PoseEstimatorType::NDT]) {
    //   return new PoseEstimator_NDT;
    } else {
      std::cerr << "[PoseEstimatorFactory::ProducePoseEstimatorPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_POSEESTIMATORFACTORY_H_