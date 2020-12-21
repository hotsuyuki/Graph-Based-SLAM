#ifndef SLAM_COSTFUNCTION_PERPENDICULARDISTANCE_H_
#define SLAM_COSTFUNCTION_PERPENDICULARDISTANCE_H_


// slam/
#include "CostFunction.h"


namespace sample_slam {

class CostFunction_PerpendicularDistance : public CostFunction {
 public:
  CostFunction_PerpendicularDistance() {}

  ~CostFunction_PerpendicularDistance() {}

  double CalcErrorDistance(const LaserPoint2D& associated_reference_laser_point,
                           const LaserPoint2D& associated_current_laser_point) override;
};

}  // namespace sample_slam


#endif  // SLAM_COSTFUNCTION_PERPENDICULARDISTANCE_H_