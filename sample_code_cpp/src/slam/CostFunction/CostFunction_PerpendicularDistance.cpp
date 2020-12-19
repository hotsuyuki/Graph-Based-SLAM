#include "CostFunction_PerpendicularDistance.h"


namespace sample_slam {

double CostFunction_PerpendicularDistance::CalcErrorDistance(const LaserPoint2D& associated_reference_laser_point,
                                                             const LaserPoint2D& associated_current_laser_point) {
  double dx = associated_current_laser_point.x - associated_reference_laser_point.x;
  double dy = associated_current_laser_point.y - associated_reference_laser_point.y;

  double perpendicular_error_distance = dx * associated_reference_laser_point.normal_x
                                          + dy * associated_reference_laser_point.normal_y;  // [m]
  
  return perpendicular_error_distance;
}

}  // namespace sample_slam