cmake_minimum_required(VERSION 3.12)

include(${CMAKE_SOURCE_DIR}/cmake/DownloadProject/DownloadProject.cmake)
download_project(
  PROJ googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG master
  UPDATE_DISCONNECTED 1
)

add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

add_executable(utility_test
  ./MyUtility_test.cpp
  ./LaserPoint2D_test.cpp
  ./Pose2D_test.cpp
)
target_compile_features(utility_test PRIVATE cxx_std_11)

target_link_libraries(utility_test
  utility
  gtest_main
)

include(GoogleTest)
gtest_add_tests(TARGET utility_test)