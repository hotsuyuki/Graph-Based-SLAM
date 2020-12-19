#ifndef SLAM_REFERENCESCANMAKER_H_
#define SLAM_REFERENCESCANMAKER_H_


// slam/
#include "PointCloudMap.h"
// utility/
#include "Scan2D.h"


namespace sample_slam {

class ReferenceScanMaker {
 public:
  ReferenceScanMaker()
    : point_cloud_map_ptr_(nullptr) {}

  virtual ~ReferenceScanMaker() {}

  void SetPointCloudMapPtr(PointCloudMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  const Scan2D& GetReferenceScan() const {
    return reference_scan_;
  }

  virtual void MakeReferenceScan() = 0;


 protected:
  PointCloudMap* point_cloud_map_ptr_;  // Pointer to abstract class, which is customizable

  Scan2D reference_scan_;
};

}  // namespace sample_slam


#endif  // SLAM_REFERENCESCANMAKER_H_