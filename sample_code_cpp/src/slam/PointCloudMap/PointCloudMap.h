#ifndef SLAM_POINTCLOUDMAP_H_
#define SLAM_POINTCLOUDMAP_H_


#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"


namespace sample_slam {

// Abstract class of `PointCloudMap`
class PointCloudMap {
 public:
  PointCloudMap() : max_num_point_(1000000), num_point_threshold_(1) {
    global_map_.reserve(max_num_point_);
  }

  virtual ~PointCloudMap() {}

  void SetNumPointThreshold(int num_point_threshold) {
    num_point_threshold_ = num_point_threshold;
  }

  void SetLastPose(const Pose2D& last_pose) {
    last_pose_ = last_pose;
  }

  void SetLastScan(const Scan2D& last_scan) {
    last_scan_ = last_scan;
  }

  const std::vector<LaserPoint2D>& GetGlobalMap() const {
    return global_map_;
  }

  const std::vector<LaserPoint2D>& GetLocalMap() const {
    return local_map_;
  }

  const std::vector<Pose2D>& GetPoses() const {
    return poses_;
  }

  const Pose2D& GetLastPose() const {
    return last_pose_;
  }

  const Scan2D& GetLastScan() const {
    return last_scan_;
  }

  virtual void AddPose(const Pose2D& pose) = 0;

  virtual void AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) = 0;

  virtual void MakeGlobalMap() = 0;

  virtual void MakeLocalMap() = 0;

  virtual void RemakeMap(const std::vector<Pose2D>& adjusted_poses) = 0;


 protected:
  std::vector<LaserPoint2D> global_map_;
  std::vector<LaserPoint2D> local_map_;

  std::vector<Pose2D> poses_;
  Pose2D last_pose_;
  Scan2D last_scan_;

  int max_num_point_;
  int num_point_threshold_;  // for `PointCloudMap_GridTable` and `PointCloudMap_SubMap`
};

}  // namespace sample_slam


#endif  // SLAM_POINTCLOUDMAP_H_