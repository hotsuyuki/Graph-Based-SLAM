#ifndef SLAM_REFERENCESCANMAKER_LASTSCAN_H_
#define SLAM_REFERENCESCANMAKER_LASTSCAN_H_


#include "ReferenceScanMaker.h"


namespace sample_slam {

// Concrete class of `ReferenceScanMaker` that uses the last scan as a reference scan
class ReferenceScanMaker_LastScan : public ReferenceScanMaker {
 public:
  ReferenceScanMaker_LastScan() {}

  ~ReferenceScanMaker_LastScan() {}

  void MakeReferenceScan() override;
};

}  // namespace sample_slam


#endif  // SLAM_REFERENCESCANMAKER_LASTSCAN_H_