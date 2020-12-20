#ifndef LAUNCHER_POINTCLOUDMAPFACTORY_H_
#define LAUNCHER_POINTCLOUDMAPFACTORY_H_


#include <iostream>
#include <map>
#include <string>

#include "PointCloudMap.h"
#include "PointCloudMap_EntirePoint.h"
#include "PointCloudMap_GridTable.h"
#include "PointCloudMap_SubMap.h"


namespace sample_slam {

enum class PointCloudMapType {
  ENTIRE_POINT = 0,
  GRID_TABLE = 1,
  SUB_MAP = 2,
};


class PointCloudMapFactory {
 public:
  PointCloudMapFactory() {}

  ~PointCloudMapFactory() {}

  static PointCloudMap* ProducePointCloudMapPtr(const std::string& chars) {
    std::map<PointCloudMapType, std::string> point_cloud_map_type = {
      { PointCloudMapType::ENTIRE_POINT, "entire_point" },
      { PointCloudMapType::GRID_TABLE, "grid_table" },
      { PointCloudMapType::SUB_MAP, "sub_map" },
    };

    if (chars == point_cloud_map_type[PointCloudMapType::ENTIRE_POINT]) {
      return new PointCloudMap_EntirePoint;
    } else if (chars == point_cloud_map_type[PointCloudMapType::GRID_TABLE]) {
      return new PointCloudMap_GridTable;
    } else if (chars == point_cloud_map_type[PointCloudMapType::SUB_MAP]) {
      return new PointCloudMap_SubMap;
    } else {
      std::cerr << "[PointCloudMapFactory::ProducePointCloudMapPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }

  static PointCloudMap* ProducePointCloudMapPtr(const PointCloudMapType& enumerate) {
    if (enumerate == PointCloudMapType::ENTIRE_POINT) {
      return new PointCloudMap_EntirePoint;
    } else if (enumerate == PointCloudMapType::GRID_TABLE) {
      return new PointCloudMap_GridTable;
    } else if (enumerate == PointCloudMapType::SUB_MAP) {
      return new PointCloudMap_SubMap;
    } else {
      std::cerr << "[PointCloudMapFactory::ProducePointCloudMapPtr()] Error: Invalid input \n\n";
      return nullptr;
    }
  }
};

}  // namespace sample_slam

#endif  // LAUNCHER_POINTCLOUDMAPFACTORY_H_