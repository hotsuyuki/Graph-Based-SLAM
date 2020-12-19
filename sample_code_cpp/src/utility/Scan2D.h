#ifndef UTILITY_SCAN2D_H_
#define UTILITY_SCAN2D_H_


#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

struct Scan2D {
  static constexpr double MIN_SCAN_RANGE = 0.1;
  static constexpr double MAX_SCAN_RANGE = 6.0;

  int id;
  Pose2D pose;
  std::vector<LaserPoint2D> laser_points;

  Scan2D() : id(-1) {}

  Scan2D(int _id) : id(_id) {}
};

}  // namespace sample_slam


#endif  // UTILITY_SCAN2D_H_