cmake_minimum_required(VERSION 3.12)

project(launcher_lib
  VERSION 1.0.0
  DESCRIPTION "Launcher library"
  LANGUAGES CXX
)

add_library(launcher STATIC
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/FrameworkFactory.cpp
  ################
  ${PROJECT_SOURCE_DIR}/MapDrawer/MapDrawer.cpp
  ${PROJECT_SOURCE_DIR}/SensorDataReader/SensorDataReader.cpp
  ${PROJECT_SOURCE_DIR}/SlamLauncher/SlamLauncher.cpp
)
target_compile_features(launcher PRIVATE cxx_std_11)

target_include_directories(launcher PUBLIC
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/CostFunctionFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/DataAssociatorFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/PointCloudMapFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/PoseEstimatorFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/PoseOptimizerFactory/
  ${PROJECT_SOURCE_DIR}/FrameworkFactory/ReferenceScanMakerFactory/
  ################
  ${PROJECT_SOURCE_DIR}/MapDrawer/
  ${PROJECT_SOURCE_DIR}/SensorDataReader/
  ${PROJECT_SOURCE_DIR}/SlamLauncher/
)

target_link_libraries(launcher
  slam
  utility
)