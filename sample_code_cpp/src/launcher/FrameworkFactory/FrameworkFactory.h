#ifndef LAUNCHER_FRAMEWORKFACTORY_H_
#define LAUNCHER_FRAMEWORKFACTORY_H_


#include <string>

// launcher/
#include "SlamLauncher.h"
// slam/
#include "CostFunction.h"
#include "DataAssociator.h"
#include "PointCloudMap.h"
#include "PoseEstimator.h"
#include "PoseOptimizer.h"
#include "ReferenceScanMaker.h"

namespace sample_slam {

class FrameworkFactory {
 public:
  FrameworkFactory()
    : point_cloud_map_ptr_(nullptr),
      reference_scan_maker_ptr_(nullptr),
      pose_estimator_ptr_(nullptr),
      data_associator_ptr_(nullptr),
      pose_optimizer_ptr_(nullptr),
      cost_function_ptr_(nullptr),
      is_scan_preprocess_(false),
      is_odometry_fusion_(false),
      is_loop_closure_(false) {}

  ~FrameworkFactory() {
    delete point_cloud_map_ptr_;
    delete reference_scan_maker_ptr_;
    delete pose_estimator_ptr_;
    delete data_associator_ptr_;
    delete pose_optimizer_ptr_;
    delete cost_function_ptr_;
  }

  SlamLauncher* GetSlamLauncherPtr() {
    return &slam_launcher_;
  }

  void CustomizeFramework(std::string framework_config_filename);


 private:
  PointCloudMap* point_cloud_map_ptr_;  // Pointer to abstract class, which is customizable
  ReferenceScanMaker* reference_scan_maker_ptr_;  // Pointer to abstract class, which is customizable
  PoseEstimator* pose_estimator_ptr_;  // Pointer to abstract class, which is customizable
  DataAssociator* data_associator_ptr_;  // Pointer to abstract class, which is customizable
  PoseOptimizer* pose_optimizer_ptr_;  // Pointer to abstract class, which is customizable
  CostFunction* cost_function_ptr_;  // Pointer to abstract class, which is customizable
  bool is_scan_preprocess_;
  bool is_odometry_fusion_;
  bool is_loop_closure_;

  SlamLauncher slam_launcher_;
};

}  // namespace sample_slam


#endif  // LAUNCHER_FRAMEWORKFACTORY_H_