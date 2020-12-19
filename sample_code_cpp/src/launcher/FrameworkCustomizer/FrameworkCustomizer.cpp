#include "FrameworkCustomizer.h"

#include <algorithm>
#include <fstream>
#include <iostream>

// slam/
#include "CostFunction_EuclideanDistance.h"
#include "CostFunction_PerpendicularDistance.h"
#include "DataAssociator_LinearSearch.h"
#include "DataAssociator_GridTable.h"
#include "PointCloudMap_EntirePoint.h"
#include "PointCloudMap_GridTable.h"
#include "PointCloudMap_SubMap.h"
#include "PoseEstimator_ICP.h"
// #include "PoseEstimator_NDT.h"
#include "PoseOptimizer_GradientDescent.h"
#include "PoseOptimizer_LineSearch.h"
#include "ReferenceScanMaker_LastScan.h"
#include "ReferenceScanMaker_LocalMap.h"
#include "LoopDetector.h"
#include "ScanMatcher.h"
#include "SlamFrontend.h"


namespace sample_slam {

#define COMMENT_PREFIX_SEMICOLON ';'
#define COMMENT_PREFIX_SHARP '#'
#define DELIMINATER_SYMBOL '='

#define POINT_CLOUD_MAP_TYPE "point_cloud_map_type"
#define ENTIRE_POINT "entire_point"
#define GRID_TABLE "grid_table"
#define SUB_MAP "sub_map"

#define REFERENCE_SCAN_MAKER_TYPE "reference_scan_maker_type"
#define LAST_SCAN "last_scan"
#define LOCAL_MAP "local_map"

#define POSE_ESTIMATOR_TYPE "pose_estimator_type"
#define ICP "icp"
// #define NDT "ndt"

#define DATA_ASSOCIATOR_TYPE "data_associator_type"
#define LINEAR_SEARCH "linear_search"
#define GRID_TABLE "grid_table"

#define POSE_OPTIMIZER_TYPE "pose_optimizer_type"
#define GRADIENT_DESCENT "gradient_descent"
#define LINE_SEARCH "line_search"

#define COST_FUNCTION_TYPE "cost_function_type"
#define EUCLIDEAN_DISTANCE "euclidean_distance"
#define PERPENDICULAR_DISTANCE "perpendicular_distance"

#define IS_SCAN_PREPROCESS "is_scan_preprocess"
#define IS_ODOMETRY_FUSION "is_odometry_fusion"
#define IS_LOOP_CLOSURE "is_loop_closure"


void FrameworkCustomizer::CustomizeFramework(std::string framework_config_filename) {
  std::ifstream framework_config_ifs(framework_config_filename, std::ios::in);
  if (!framework_config_ifs.is_open()) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] ";
    std::cerr << "Error: framework_config_ifs.is_open() = " << framework_config_ifs.is_open() << "\n\n";
    return;
  }

  // Parses framework config file
  // https://www.walletfox.com/course/parseconfigfile.php
  std::string line;
  while (std::getline(framework_config_ifs, line)) {
    // Removes whitespaces
    line.erase(std::remove_if(line.begin(), line.end(), isspace), line.end());

    // Skips if the line is comment (that starts from ';' or '#) or empty
    if (line[0] == COMMENT_PREFIX_SEMICOLON || line[0] == COMMENT_PREFIX_SHARP || line.empty()) {
      continue;
    }

    // Parses into key and value deliminated by '='
    std::size_t equal_pos = line.find(DELIMINATER_SYMBOL);
    std::string key = line.substr(0, equal_pos);
    std::string value = line.substr(equal_pos + 1);

    std::cout << "[FrameworkCustomizer::CustomizeFramework()] ";
    std::cout << "key = " << key << ", value = " << value << "\n";

    if (key == POINT_CLOUD_MAP_TYPE)
    {
      if (value == ENTIRE_POINT) {
        point_cloud_map_ptr_ = new PointCloudMap_EntirePoint;
      } else if (value == GRID_TABLE) {
        point_cloud_map_ptr_ = new PointCloudMap_GridTable;
      } else if (value == SUB_MAP) {
        point_cloud_map_ptr_ = new PointCloudMap_SubMap;
      } else {
        point_cloud_map_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == REFERENCE_SCAN_MAKER_TYPE)
    {
      if (value == LAST_SCAN) {
        reference_scan_maker_ptr_ = new ReferenceScanMaker_LastScan;
      } else if (value == LOCAL_MAP) {
        reference_scan_maker_ptr_ = new ReferenceScanMaker_LocalMap;
      } else {
        reference_scan_maker_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == POSE_ESTIMATOR_TYPE)
    {
      if (value == ICP) {
        pose_estimator_ptr_ = new PoseEstimator_ICP;
      // } else if (value == NDT) {
        // pose_estimator_ptr_ = new PoseEstimator_NDT;
      } else {
        pose_estimator_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == DATA_ASSOCIATOR_TYPE)
    {
      if (value == LINEAR_SEARCH) {
        data_associator_ptr_ = new DataAssociator_LinearSearch;
      } else if (value == GRID_TABLE) {
        data_associator_ptr_ = new DataAssociator_GridTable;
      } else {
        data_associator_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == POSE_OPTIMIZER_TYPE)
    {
      if (value == GRADIENT_DESCENT) {
        pose_optimizer_ptr_ = new PoseOptimizer_GradientDescent;
      } else if (value == LINE_SEARCH) {
        pose_optimizer_ptr_ = new PoseOptimizer_LineSearch;
      } else {
        pose_optimizer_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == COST_FUNCTION_TYPE)
    {
      if (value == EUCLIDEAN_DISTANCE) {
        cost_function_ptr_ = new CostFunction_EuclideanDistance;
      } else if (value == PERPENDICULAR_DISTANCE) {
        cost_function_ptr_ = new CostFunction_PerpendicularDistance;
      } else {
        cost_function_ptr_ = nullptr;
        std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: Invalid key and/or value... \n\n";
      }
    }
    else if (key == IS_SCAN_PREPROCESS)
    {
      if (std::stoi(value) == 1) {
        is_scan_preprocess_ = true;
      } else {
        is_scan_preprocess_ = false;
      }
    }
    else if (key == IS_ODOMETRY_FUSION)
    {
      if (std::stoi(value) == 1) {
        is_odometry_fusion_ = true;
      } else {
        is_odometry_fusion_ = false;
      }
    }
    else if (key == IS_LOOP_CLOSURE)
    {
      if (std::stoi(value) == 1) {
        is_loop_closure_ = true;

        delete point_cloud_map_ptr_;
        point_cloud_map_ptr_ = new PointCloudMap_SubMap;

        delete data_associator_ptr_;
        data_associator_ptr_ = new DataAssociator_GridTable;

        delete cost_function_ptr_;
        cost_function_ptr_ = new CostFunction_PerpendicularDistance;
      } else {
        is_loop_closure_ = false;
      }
    }
  }

  std::cout << "\n";

  if (point_cloud_map_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: point_cloud_map_ptr_ == nullptr... \n\n";
  }
  if (reference_scan_maker_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: reference_scan_maker_ptr_ == nullptr... \n\n";
  }
  if (pose_estimator_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: pose_estimator_ptr_ == nullptr... \n\n";
  }
  if (data_associator_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: data_associator_ptr_ == nullptr... \n\n";
  }
  if (pose_optimizer_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: pose_optimizer_ptr_ == nullptr... \n\n";
  }
  if (cost_function_ptr_ == nullptr) {
    std::cerr << "[FrameworkCustomizer::CustomizeFramework()] Error: cost_function_ptr_ == nullptr... \n\n";
  }

  slam_launcher_.SetPointCloudMapPtr(point_cloud_map_ptr_);
  slam_launcher_.SetIsScanPreprocess(is_scan_preprocess_);

  SlamFrontend* slam_frontend_ptr = slam_launcher_.GetSlamFrontendPtr();
  slam_frontend_ptr->SetPointCloudMapPtr(point_cloud_map_ptr_);
  slam_frontend_ptr->SetIsLoopClosure(is_loop_closure_);

  ScanMatcher* scan_mather_ptr = slam_frontend_ptr->GetScanMatcherPtr();
  scan_mather_ptr->SetPointCloudMapPtr(point_cloud_map_ptr_);
  scan_mather_ptr->SetReferenceScanMakerPtr(reference_scan_maker_ptr_);
  scan_mather_ptr->SetPoseEstimatorPtr(pose_estimator_ptr_);
  scan_mather_ptr->SetIsScanPreprocess(is_scan_preprocess_);
  scan_mather_ptr->SetIsOdometryFusion(is_odometry_fusion_);

  ReferenceScanMaker* reference_scan_maker_ptr = scan_mather_ptr->GetReferenceScanMakerPtr();
  reference_scan_maker_ptr->SetPointCloudMapPtr(point_cloud_map_ptr_);

  PoseEstimator* pose_estimator_ptr = scan_mather_ptr->GetPoseEstimatorPtr();
  pose_estimator_ptr->SetDataAssociatorPtr(data_associator_ptr_);
  pose_estimator_ptr->SetPoseOptimizerPtr(pose_optimizer_ptr_);

  PoseOptimizer* pose_optimizer_ptr = pose_estimator_ptr->GetPoseOptimizerPtr();
  pose_optimizer_ptr->SetCostFunctionPtr(cost_function_ptr_);

  if (is_loop_closure_) {
    LoopDetector* loop_detector_ptr = slam_frontend_ptr->GetLoopDetectorPtr();
    loop_detector_ptr->SetPointCloudMapPtr((PointCloudMap_SubMap*)point_cloud_map_ptr_);
    loop_detector_ptr->SetDataAssociatorPtr((DataAssociator_GridTable*)data_associator_ptr_);
    loop_detector_ptr->SetCostFunctionPtr((CostFunction_PerpendicularDistance*)cost_function_ptr_);
    loop_detector_ptr->SetPoseEstimatorPtr(pose_estimator_ptr_);

    SlamBackend* slam_backend_ptr = slam_frontend_ptr->GetSlamBackendPtr();
    slam_backend_ptr->SetPointCloudMapPtr((PointCloudMap_SubMap*)point_cloud_map_ptr_);
  }

  return;
}

}  // namespace sample_slam