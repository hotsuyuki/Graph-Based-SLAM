cmake_minimum_required(VERSION 3.12)

project(sample_slam
  VERSION 1.0.0
  DESCRIPTION "Sample SLAM: Scan matching as a front-end, Graph based SLAM as a back-end"
  LANGUAGES CXX
)

add_subdirectory(${CMAKE_SOURCE_DIR}/launcher/)
add_subdirectory(${CMAKE_SOURCE_DIR}/slam/)
add_subdirectory(${CMAKE_SOURCE_DIR}/utility/)

enable_testing()
# add_subdirectory(${CMAKE_SOURCE_DIR}/test/launcher/)
# add_subdirectory(${CMAKE_SOURCE_DIR}/test/slam/)
add_subdirectory(${CMAKE_SOURCE_DIR}/test/utility/)

add_executable(main ${CMAKE_SOURCE_DIR}/main.cpp)
target_compile_features(main PRIVATE cxx_std_11)

target_include_directories(main PRIVATE
  ${CMAKE_SOURCE_DIR}/cxxopts/
)

target_link_libraries(main
  launcher
)