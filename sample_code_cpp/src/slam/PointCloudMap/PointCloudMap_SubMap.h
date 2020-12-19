#ifndef SLAM_POINTCLOUDMAP_SUBMAP_H_
#define SLAM_POINTCLOUDMAP_SUBMAP_H_


#include <vector>

// slam/
#include "PointCloudMap.h"
// utility/
#include "GridTable2D.h"
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

struct SubMap {
  double cumulative_distance_start;
  int scan_id_start;
  int scan_id_end;
  std::vector<LaserPoint2D> map_laser_points;

  void AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) {
    for (std::size_t i = 0; i < laser_points.size(); ++i) {
      map_laser_points.emplace_back(laser_points[i]);
    }

    return;
  }

  std::vector<LaserPoint2D> SubsampleMapPoints(int num_point_threshold) {
    GridTable2D grid_table;
    for (std::size_t i = 0; i < map_laser_points.size(); ++i) {
      grid_table.AddLaserPoint(map_laser_points[i]);
    }

    std::vector<LaserPoint2D> subsampled_map_laser_points;
    grid_table.MakeCellPoints(num_point_threshold, subsampled_map_laser_points);

    std::cout << "[SubMap::SubsampleMapPoints()] "
              << "map_laser_points.size() = " << map_laser_points.size()
              << ", subsampled_map_laser_points.size() = " << subsampled_map_laser_points.size() << "\n";
    
    return subsampled_map_laser_points;
  }
};


// Concrete class of `PointCloudMap` that stores laser points in a grid table
class PointCloudMap_SubMap : public PointCloudMap {
 public:
  PointCloudMap_SubMap()
    : cumulative_distance_threshold_(10.0),
      cumulative_distance_(0.0) {
    SubMap sub_map;
    sub_maps_.emplace_back(sub_map);
  }

  virtual ~PointCloudMap_SubMap() {}

  const std::vector<SubMap>& GetSubMaps() const {
    return sub_maps_;
  }

  double GetCumulativeDistance() const {
    return cumulative_distance_;
  }

  void AddPose(const Pose2D& pose) override;

  void AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) override;

  void MakeGlobalMap() override;

  void MakeLocalMap() override;

  void RemakeMap(const std::vector<Pose2D>& adjusted_poses) override;


 private:
  double cumulative_distance_threshold_;
  double cumulative_distance_;
  std::vector<SubMap> sub_maps_;
};

}  // namespace sample_slam


#endif  // SLAM_POINTCLOUDMAP_SUBMAP_H_