#include "PointCloudMap_SubMap.h"


namespace sample_slam {

void PointCloudMap_SubMap::AddPose(const Pose2D& pose) {
  if (poses_.size() == 0) {
    cumulative_distance_ += hypot(pose.x, pose.y);
  } else if (0 < poses_.size()) {
    Pose2D last_pose = poses_.back();
    double dx = pose.x - last_pose.x;
    double dy = pose.y - last_pose.y;
    cumulative_distance_ += hypot(dx, dy);
  }

  poses_.emplace_back(pose);

  return;
}


void PointCloudMap_SubMap::AddLaserPoints(const std::vector<LaserPoint2D>& laser_points) {
  SubMap* curr_sub_map_ptr = &(sub_maps_.back());

  double delta_cumulative_distance = cumulative_distance_ - curr_sub_map_ptr->cumulative_distance_start;

  if (cumulative_distance_threshold_ <= delta_cumulative_distance) {
    curr_sub_map_ptr->scan_id_end = poses_.size() - 1;
    curr_sub_map_ptr->map_laser_points = curr_sub_map_ptr->SubsampleMapPoints(num_point_threshold_);

    SubMap new_sub_map;
    new_sub_map.cumulative_distance_start = cumulative_distance_;
    new_sub_map.scan_id_start = poses_.size();
    new_sub_map.AddLaserPoints(laser_points);

    sub_maps_.emplace_back(new_sub_map);
  } else {
    curr_sub_map_ptr->AddLaserPoints(laser_points);
  }

  return;
}


void PointCloudMap_SubMap::MakeGlobalMap() {
  global_map_.clear();
  local_map_.clear();

  for (std::size_t i = 0; i < (sub_maps_.size() - 1); ++i) {
    std::vector<LaserPoint2D>& sub_map_i_map_laser_points = sub_maps_[i].map_laser_points;

    for (std::size_t j = 0; j < sub_map_i_map_laser_points.size(); ++j) {
      global_map_.emplace_back(sub_map_i_map_laser_points[j]);
    }

    // Make local map as well using the last sub map
    if (i == (sub_maps_.size() - 2)) {
      for (std::size_t j = 0; j < sub_map_i_map_laser_points.size(); ++j) {
        local_map_.emplace_back(sub_map_i_map_laser_points[j]);
      }
    }
  }

  SubMap* curr_sub_map_ptr = &(sub_maps_.back());
  std::vector<LaserPoint2D> subsampled_map_laser_points = curr_sub_map_ptr->SubsampleMapPoints(num_point_threshold_);
  for (std::size_t i = 0; i < subsampled_map_laser_points.size(); ++i) {
    global_map_.emplace_back(subsampled_map_laser_points[i]);
    local_map_.emplace_back(subsampled_map_laser_points[i]);
  }

  std::cout << "[PointCloudMap_SubMap::MakeGlobalMap()] \n"
            << "curr_sub_map_ptr->cumulative_distance_start = " << curr_sub_map_ptr->cumulative_distance_start
            << ",cumulative_distance_ = " << cumulative_distance_
            << ", subsampled_map_laser_points.size() = " << subsampled_map_laser_points.size() << "\n"
            << "sub_maps_.size() = " << sub_maps_.size() << ", global_map_.size() = " << global_map_.size() << "\n";

  return;
}


void PointCloudMap_SubMap::MakeLocalMap() {
  local_map_.clear();

  if (2 <= sub_maps_.size()) {
    SubMap* last_sub_map_ptr = &(sub_maps_[sub_maps_.size() - 2]);
    for (std::size_t i = 0; i < last_sub_map_ptr->map_laser_points.size(); ++i) {
      local_map_.emplace_back(last_sub_map_ptr->map_laser_points[i]);
    }
  }

  SubMap* curr_sub_map_ptr = &(sub_maps_.back());
  std::vector<LaserPoint2D> subsampled_map_laser_points = curr_sub_map_ptr->SubsampleMapPoints(num_point_threshold_);
  for (std::size_t i = 0; i < subsampled_map_laser_points.size(); ++i) {
    local_map_.emplace_back(subsampled_map_laser_points[i]);
  }

  std::cout << "[PointCloudMap_SubMap::MakeLocalMap()] "
            << "local_map_.size() = " << local_map_.size() << "\n";

  return;
}


void PointCloudMap_SubMap::RemakeMap(const std::vector<Pose2D>& adjusted_poses) {
  for (std::size_t i = 0; i < sub_maps_.size(); ++i) {
    std::vector<LaserPoint2D>& sub_map_i_map_laser_points = sub_maps_[i].map_laser_points;
    for (std::size_t j = 0; j < sub_map_i_map_laser_points.size(); ++j) {
      LaserPoint2D& laser_point_j = sub_map_i_map_laser_points[j];
      if (poses_.size() <= laser_point_j.id) {
        continue;
      }

      const Pose2D& old_pose = poses_[laser_point_j.id];
      const Pose2D& new_pose = adjusted_poses[laser_point_j.id];

      LaserPoint2D old_local_laser_point_j;
      old_pose.GlobalCoord2LocalCoord(laser_point_j, old_local_laser_point_j);
      old_pose.InverseRotateNormalVector(laser_point_j, old_local_laser_point_j);

      LaserPoint2D new_global_laser_point_j;
      new_pose.LocalCoord2GlobalCoord(old_local_laser_point_j, new_global_laser_point_j);
      new_pose.RotateNormalVector(old_local_laser_point_j, new_global_laser_point_j);

      laser_point_j.x = new_global_laser_point_j.x;
      laser_point_j.y = new_global_laser_point_j.y;
      laser_point_j.normal_x = new_global_laser_point_j.normal_x;
      laser_point_j.normal_y = new_global_laser_point_j.normal_y;
    }
  }

  MakeGlobalMap();

  for (std::size_t i = 0; i < poses_.size(); ++i) {
    poses_[i] = adjusted_poses[i];
  }
  last_pose_ = adjusted_poses.back();

  return;
}

}  // namespace sample_slam