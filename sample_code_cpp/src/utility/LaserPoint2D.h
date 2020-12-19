#ifndef UTILITY_LASERPOINT2D_H_
#define UTILITY_LASERPOINT2D_H_


#include <cmath>

// utility/
#include "MyUtility.h"


namespace sample_slam {

enum PointType {
  UNKNOWN = 0,
  LINE = 1,
  CORNER = 2,
  ISOLATE = 3
};


struct LaserPoint2D {
  int id;
  double x, y;
  double normal_x, normal_y;  // Normal vector of the laser point
  PointType point_type;

  LaserPoint2D()
      : id(-1), x(0.0), y(0.0), normal_x(0.0), normal_y(0.0), point_type(UNKNOWN) {}

  LaserPoint2D(int _id, double _x, double _y)
      : id(_id), x(_x), y(_y), normal_x(0.0), normal_y(0.0), point_type(UNKNOWN) {}

  LaserPoint2D(int _id, double _x, double _y, double _normal_x, double _normal_y)
      : id(_id), x(_x), y(_y), normal_x(_normal_x), normal_y(_normal_y), point_type(UNKNOWN) {}

  LaserPoint2D(int _id, double _x, double _y, double _normal_x, double _normal_y, PointType _point_type)
      : id(_id), x(_x), y(_y), normal_x(_normal_x), normal_y(_normal_y), point_type(_point_type) {}

  void RangeYaw2XY(double range, double yaw) {
    x = range * cos(yaw);
    y = range * sin(yaw);
  }
};

}  // namespace sample_slam


#endif  // UTILITY_LASERPOINT2D_H_