#ifndef SLAM_REFERENCESCANMAKER_LOCALMAP_H_
#define SLAM_REFERENCESCANMAKER_LOCALMAP_H_


#include "ReferenceScanMaker.h"


namespace sample_slam {

// Concrete class of `ReferenceScanMaker` that uses the local map as a reference scan
class ReferenceScanMaker_LocalMap : public ReferenceScanMaker {
 public:
  ReferenceScanMaker_LocalMap() {}

  ~ReferenceScanMaker_LocalMap() {}

  void MakeReferenceScan() override;
};

}  // namespace sample_slam


#endif  // SLAM_REFERENCESCANMAKER_LOCALMAP_H_