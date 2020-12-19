#ifndef SLAM_SLAMBACKEND_H_
#define SLAM_SLAMBACKEND_H_


#include <vector>

#include <Eigen/Core>

//slam/
#include "PointCloudMap_SubMap.h"
#include "PoseGraph.h"
// utility/
#include "Pose2D.h"


namespace sample_slam {

class SlamBackend {
 public:
  SlamBackend() {}

  ~SlamBackend() {}

  void SetPointCloudMapPtr(PointCloudMap_SubMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  void SetPoseGraphPtr(PoseGraph* pose_graph_ptr) {
    pose_graph_ptr_ = pose_graph_ptr;
  }

  void AdjustPoses(int pose_adjustment_iteration);

  void RemakeMap();


 private:
  PointCloudMap_SubMap* point_cloud_map_ptr_;
  PoseGraph* pose_graph_ptr_;

  std::vector<Pose2D> adjusted_poses_;
};

}  // namespace sample_slam


#endif  // SLAM_SLAMBACKEND_H_