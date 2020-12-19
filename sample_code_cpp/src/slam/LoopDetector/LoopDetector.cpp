#include "LoopDetector.h"

#include <iostream>


namespace sample_slam {

bool LoopDetector::DetectLoop(const Scan2D& curr_scan, const Pose2D& estimated_pose, int count,
                              LoopInfo& loop_info) {
  double cumulative_distance = 0.0;  // [m]
  Pose2D prev_pose(0.0, 0.0, 0.0);  // [m], [m], [rad]

  double min_distance = __DBL_MAX__;  // [m]
  int min_sub_map_idx;
  int min_revisit_candidate_idx;

  const std::vector<SubMap>& sub_maps = point_cloud_map_ptr_->GetSubMaps();
  const std::vector<Pose2D>& map_poses = point_cloud_map_ptr_->GetPoses();
  double map_cumulative_distance = point_cloud_map_ptr_->GetCumulativeDistance(); 

  for (std::size_t i = 0; i < sub_maps.size() - 1; ++i) {
    for (std::size_t j = sub_maps[i].scan_id_start; j <= sub_maps[i].scan_id_end; ++j) {
      cumulative_distance += hypot(map_poses[j].x - prev_pose.x, map_poses[j].y - prev_pose.y);
      if ((map_cumulative_distance - cumulative_distance) < cumulative_distance_threshold_) {
        i = sub_maps.size();  // in order to break i's loop
        break;  // in order to break j's loop
      }

      double distance = hypot(estimated_pose.x - map_poses[j].x, estimated_pose.y - map_poses[j].y);
      if (distance < min_distance) {
        min_distance = distance;
        min_sub_map_idx = i;
        min_revisit_candidate_idx = j;
      }

      prev_pose = map_poses[j];
    }
  }

  std::cout << "[LoopDetector::DetectLoop()] "
            << "min_distance = " << min_distance << ", min_distance_threshold_ = " << min_distance_threshold_
            << ", min_sub_map_idx = " << min_sub_map_idx
            << ", min_revisit_candidate_idx = " << min_revisit_candidate_idx << "\n";

  if (min_distance_threshold_ < min_distance) {
    return false;
  }

  Scan2D reference_scan;
  reference_scan.laser_points = sub_maps[min_sub_map_idx].map_laser_points;

  Pose2D revisit_pose;
  bool is_loop_detected = SearchRevisitPose(reference_scan, curr_scan, estimated_pose,
                                            revisit_pose);

  if (is_loop_detected) {
    Eigen::Matrix3f scan_match_covariance;
    pose_estimator_ptr_->CalcScanMatchCovariance(reference_scan, curr_scan, revisit_pose,
                                                 scan_match_covariance);

    loop_info.reference_scan_id = min_revisit_candidate_idx;
    loop_info.curr_scan_id = count;
    loop_info.revisit_pose = revisit_pose;
    loop_info.scan_match_covariance = scan_match_covariance;  // on absolute (map) coordinate
  }
  
  return is_loop_detected;
}


bool LoopDetector::SearchRevisitPose(const Scan2D& reference_scan, const Scan2D& curr_scan, const Pose2D& estimated_pose,
                                     Pose2D& revisit_pose) {
  data_associator_ptr_->SetReferenceLaserPoints(reference_scan.laser_points);
  data_associator_ptr_->SetCurrentLaserPoints(curr_scan.laser_points);

  std::vector<Pose2D> candidate_poses;
  for (double dx = -revisit_max_distance_; dx <= revisit_max_distance_; dx += revisit_delta_distance_) {
    double search_x = estimated_pose.x + dx;

    for (double dy = -revisit_max_distance_; dy <= revisit_max_distance_; dy += revisit_delta_distance_) {
      double search_y = estimated_pose.y + dy;

      for (double dyaw = -revisit_max_angle_; dyaw <= revisit_max_angle_; dyaw += revisit_delta_angle_) {
        double search_yaw = MyUtility::AddRadian(estimated_pose.yaw, dyaw); 

        Pose2D search_pose(search_x, search_y, search_yaw);
        double association_ratio = data_associator_ptr_->AssociateCorrespondingLaserPoints(search_pose);
        int num_used_point = data_associator_ptr_->GetAssociatedCurrentLaserPoints().size();

        if (association_ratio < association_ratio_threshold_ || num_used_point < num_used_point_threshold_) {
          continue;
        }

        cost_function_ptr_->SetAssociatedLaserPoints(data_associator_ptr_->GetAssociatedReferenceLaserPoints(),
                                                     data_associator_ptr_->GetAssociatedCurrentLaserPoints());
        
        double match_cost_tmp = cost_function_ptr_->CalcMatchCost(search_x, search_y, search_yaw);
        double matched_point_ratio = cost_function_ptr_->GetMatchedPointRatio();

        if (matched_point_ratio < matched_point_ratio_threshold_) {
          continue;
        }

        candidate_poses.emplace_back(search_pose);
      }
    }
  }

  std::cout << "[LoopDetector::SearchRevisitPose()] "
            << "candidate_poses.size() = " << candidate_poses.size() << "\n";

  if (candidate_poses.size() == 0) {
    return false;
  }

  pose_estimator_ptr_->SetReferenceScan(reference_scan);
  pose_estimator_ptr_->SetCurrentScan(curr_scan);

  double best_match_cost = __DBL_MAX__;
  Pose2D best_scan_match_pose;
  for (std::size_t i = 0; i < candidate_poses.size(); ++i) {
    pose_estimator_ptr_->EstimatePose(candidate_poses[i]);

    double matched_point_ratio = cost_function_ptr_->GetMatchedPointRatio();
    int num_used_point = pose_estimator_ptr_->GetNumUsedPoint();
    if (matched_point_ratio < 1.1*matched_point_ratio_threshold_ && num_used_point < num_used_point_threshold_) {
      continue;
    }

    double min_match_cost = pose_estimator_ptr_->GetMinMatchCost();
    if (min_match_cost < best_match_cost) {
      best_match_cost = min_match_cost;
      best_scan_match_pose = pose_estimator_ptr_->GetEstimatedPose();

      std::cout << "[LoopDetector::SearchRevisitPose()] "
                << "best_match_cost = " << best_match_cost
                << ", matched_point_ratio = " << matched_point_ratio
                << ", num_used_point = " << num_used_point << "\n";
    }
  }

  if (best_match_cost < best_match_cost_threshold_) {
    revisit_pose = best_scan_match_pose;
    return true;
  }

  return false;
}

}  // namespace sample_slam