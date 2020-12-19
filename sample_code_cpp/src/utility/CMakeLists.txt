cmake_minimum_required(VERSION 3.12)

project(utility_lib
  VERSION 1.0.0
  DESCRIPTION "Utility library"
  LANGUAGES CXX
)

find_package(Eigen3)
if(NOT EIGEN3_INCLUDE_DIR)
  set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
endif()
message(STATUS "  EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

add_library(utility STATIC
  ${PROJECT_SOURCE_DIR}/MyUtility/MyUtility.cpp
)
target_compile_features(utility PRIVATE cxx_std_11)

target_include_directories(utility PUBLIC
  ${EIGEN3_INCLUDE_DIR}/
  ${PROJECT_SOURCE_DIR}/
  ${PROJECT_SOURCE_DIR}/MyUtility/
)