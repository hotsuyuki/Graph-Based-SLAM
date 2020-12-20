#ifndef LAUNCHER_REFERENCESCANMAKERFACTORY_H_
#define LAUNCHER_REFERENCESCANMAKERFACTORY_H_


#include <iostream>
#include <map>
#include <string>

#include "ReferenceScanMaker.h"
#include "ReferenceScanMaker_LastScan.h"
#include "ReferenceScanMaker_LocalMap.h"


namespace sample_slam {

enum class ReferenceScanMakerType {
  LAST_SCAN = 0,
  LOCAL_MAP = 1,
};


class ReferenceScanMakerFactory {
 public:
  ReferenceScanMakerFactory() {}

  ~ReferenceScanMakerFactory() {}

  static ReferenceScanMaker* ProduceReferenceScanMakerPtr(const std::string& chars) {
    std::map<ReferenceScanMakerType, std::string> reference_scan_maker_type = {
      { ReferenceScanMakerType::LAST_SCAN, "last_scan" },
      { ReferenceScanMakerType::LOCAL_MAP, "local_map" },
    };

    if (chars == reference_scan_maker_type[ReferenceScanMakerType::LAST_SCAN]) {
      return new ReferenceScanMaker_LastScan;
    } else if (chars == reference_scan_maker_type[ReferenceScanMakerType::LOCAL_MAP]) {
      return new ReferenceScanMaker_LocalMap;
    } else {
      std::cerr << "[ReferenceScanMakerFactory::ProduceReferenceScanMakerPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_REFERENCESCANMAKERFACTORY_H_