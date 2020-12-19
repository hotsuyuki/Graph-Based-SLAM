cmake_minimum_required(VERSION 3.12)

project(slam_lib
  VERSION 1.0.0
  DESCRIPTION "SLAM library"
  LANGUAGES CXX
)

find_package(Boost REQUIRED)
message(STATUS "  Boost_INCLUDE_DIR = ${Boost_INCLUDE_DIR}")

add_library(slam STATIC
  ${PROJECT_SOURCE_DIR}/CostFunction/CostFunction_EuclideanDistance.cpp
  ${PROJECT_SOURCE_DIR}/CostFunction/CostFunction_PerpendicularDistance.cpp
  ########
  ${PROJECT_SOURCE_DIR}/DataAssociator/DataAssociator_LinearSearch.cpp
  ${PROJECT_SOURCE_DIR}/DataAssociator/DataAssociator_GridTable.cpp
  ########
  ${PROJECT_SOURCE_DIR}/PointCloudMap/PointCloudMap_EntirePoint.cpp
  ${PROJECT_SOURCE_DIR}/PointCloudMap/PointCloudMap_GridTable.cpp
  ${PROJECT_SOURCE_DIR}/PointCloudMap/PointCloudMap_SubMap.cpp
  ########
  ${PROJECT_SOURCE_DIR}/PoseEstimator/PoseEstimator_ICP.cpp
  # ${PROJECT_SOURCE_DIR}/PoseEstimator/PoseEstimator_NDT.cpp
  ########
  ${PROJECT_SOURCE_DIR}/PoseOptimizer/PoseOptimizer_GradientDescent.cpp
  ${PROJECT_SOURCE_DIR}/PoseOptimizer/PoseOptimizer_LineSearch.cpp
  ########
  ${PROJECT_SOURCE_DIR}/ReferenceScanMaker/ReferenceScanMaker_LastScan.cpp
  ${PROJECT_SOURCE_DIR}/ReferenceScanMaker/ReferenceScanMaker_LocalMap.cpp
  ################
  ${PROJECT_SOURCE_DIR}/CovarianceCalculator/CovarianceCalculator.cpp
  ${PROJECT_SOURCE_DIR}/LoopDetector/LoopDetector.cpp
  ${PROJECT_SOURCE_DIR}/PoseGraph/PoseGraph.cpp
  ${PROJECT_SOURCE_DIR}/ScanMatcher/ScanMatcher.cpp
  ${PROJECT_SOURCE_DIR}/ScanPreprocessor/ScanPreprocessor.cpp
  ${PROJECT_SOURCE_DIR}/SlamBackend/SlamBackend.cpp
  ${PROJECT_SOURCE_DIR}/SlamFrontend/SlamFrontend.cpp
)
target_compile_features(slam PRIVATE cxx_std_11)

target_include_directories(slam PUBLIC
  ${Boost_INCLUDE_DIR}/
  ${PROJECT_SOURCE_DIR}/CostFunction/
  ${PROJECT_SOURCE_DIR}/DataAssociator/
  ${PROJECT_SOURCE_DIR}/PointCloudMap/
  ${PROJECT_SOURCE_DIR}/PoseEstimator/
  ${PROJECT_SOURCE_DIR}/PoseOptimizer/
  ${PROJECT_SOURCE_DIR}/ReferenceScanMaker/
  ################
  ${PROJECT_SOURCE_DIR}/CovarianceCalculator/
  ${PROJECT_SOURCE_DIR}/LoopDetector/
  ${PROJECT_SOURCE_DIR}/PoseGraph/
  ${PROJECT_SOURCE_DIR}/ScanMatcher/
  ${PROJECT_SOURCE_DIR}/ScanPreprocessor/
  ${PROJECT_SOURCE_DIR}/SlamBackend/
  ${PROJECT_SOURCE_DIR}/SlamFrontend/
  ################
  ${CMAKE_SOURCE_DIR}/solver/p2o/
)

target_link_libraries(slam
  utility
)