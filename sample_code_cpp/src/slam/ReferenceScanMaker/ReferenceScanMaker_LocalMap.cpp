#include "ReferenceScanMaker_LocalMap.h"


namespace sample_slam {

void ReferenceScanMaker_LocalMap::MakeReferenceScan() {
  reference_scan_.laser_points.clear();

  const std::vector<LaserPoint2D>& local_map = point_cloud_map_ptr_->GetLocalMap();
  for (std::size_t i = 0; i < local_map.size(); ++i) {
    reference_scan_.laser_points.emplace_back(local_map[i]);
  }

  return;
}

}  // namespace sample_slam