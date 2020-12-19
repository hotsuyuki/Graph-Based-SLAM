#include "CostFunction_EuclideanDistance.h"


namespace sample_slam {

double CostFunction_EuclideanDistance::CalcErrorDistance(const LaserPoint2D& associated_reference_laser_point,
                                                         const LaserPoint2D& associated_current_laser_point) {
  double dx = associated_current_laser_point.x - associated_reference_laser_point.x;
  double dy = associated_current_laser_point.y - associated_reference_laser_point.y;

  double euclidean_error_distance = hypot(dx, dy);  // [m]

  return euclidean_error_distance;
}

}  // namespace sample_slam