#include "ScanMatcher.h"

#include <iostream>
#include <vector>

#include <Eigen/Core>

// slam/
#include "ScanPreprocessor.h"
// utility/
#include "LaserPoint2D.h"


namespace sample_slam {

bool ScanMatcher::MatchScan(const Scan2D& scan, int count) {
  Scan2D curr_scan(scan);

  if (is_scan_preprocess_) {
    ScanPreprocessor scan_preprocessor;
    scan_preprocessor.ResampleLaserPoints(curr_scan.laser_points);
    scan_preprocessor.CalcNormalVectors(curr_scan.laser_points);
  }

  if (count == 0) {
    GrowMap(curr_scan, Pose2D(0.0, 0.0, 0.0), count);
    prev_scan_ = curr_scan;

    return true;
  }

  Pose2D curr_scan_pose(curr_scan.pose);
  Pose2D prev_scan_pose(prev_scan_.pose);
  Pose2D odometry_pose_relative = curr_scan_pose - prev_scan_pose;  // Inverse compounding operator
  
  const Pose2D& last_pose = point_cloud_map_ptr_->GetLastPose();
  Pose2D odometry_pose = last_pose + odometry_pose_relative;  // Compounding operator

  reference_scan_maker_ptr_->MakeReferenceScan();
  const Scan2D& reference_scan = reference_scan_maker_ptr_->GetReferenceScan();

  pose_estimator_ptr_->SetReferenceScan(reference_scan);
  pose_estimator_ptr_->SetCurrentScan(curr_scan);

  auto start_EstimatePose = std::chrono::high_resolution_clock::now();
  pose_estimator_ptr_->EstimatePose(odometry_pose);
  auto end_EstimatePose = std::chrono::high_resolution_clock::now();
  auto duration_EstimatePose = std::chrono::duration_cast<std::chrono::milliseconds>(end_EstimatePose - start_EstimatePose);
  std::cout << "duration_EstimatePose.count() = " << duration_EstimatePose.count() << " [ms] \n";

  Pose2D scan_match_pose(pose_estimator_ptr_->GetEstimatedPose());
  double min_match_cost = pose_estimator_ptr_->GetMinMatchCost();
  int num_used_point = pose_estimator_ptr_->GetNumUsedPoint();

  bool is_scan_match_success;
  Pose2D estimated_pose;
  if (min_match_cost < match_cost_threshold_ && num_used_point_threshold_ < num_used_point) {
    is_scan_match_success = true;
    estimated_pose = scan_match_pose;  // Adopts the scan-matched pose
  } else {
    is_scan_match_success = false;
    estimated_pose = odometry_pose;  // Adopts the odometry-only pose
  }

  std::cout << "[ScanMatcher::MatchScan()] is_scan_match_success = " << is_scan_match_success << " : "
            << "min_match_cost = " << min_match_cost << ", num_used_point = " << num_used_point << "\n";

  if (is_odometry_fusion_) {
    if (is_scan_match_success) {
      Pose2D fused_pose;
      Eigen::Matrix3f fused_covariance;
      pose_estimator_ptr_->OdometryFusion(reference_scan, curr_scan, scan_match_pose,
                                          last_pose, odometry_pose_relative, odometry_pose,
                                          fused_pose, fused_covariance);

      estimated_pose = fused_pose;  // Adopts the scan-odometry-fused pose
      move_covariance_ = fused_covariance;
    } else {
      Eigen::Matrix3f odometry_covariance;
      pose_estimator_ptr_->CalcOdometryCovariance(last_pose, odometry_pose_relative,
                                                  odometry_covariance);

      estimated_pose = odometry_pose;  // Adopts the odometry-only pose
      move_covariance_ = odometry_covariance;
    }
  }

  GrowMap(curr_scan, estimated_pose, count);
  prev_scan_ = curr_scan;

  return is_scan_match_success;
}


void ScanMatcher::GrowMap(const Scan2D& curr_scan, const Pose2D& estimated_pose, int count) {
  std::vector<LaserPoint2D> global_laser_points;
  for (std::size_t i = 0; i < curr_scan.laser_points.size(); ++i) {
    const LaserPoint2D& laser_point = curr_scan.laser_points[i];
    if (laser_point.point_type == ISOLATE) {
      continue;
    }

    LaserPoint2D map_laser_point;
    map_laser_point.id = count; 
    map_laser_point.point_type = laser_point.point_type; 

    estimated_pose.LocalCoord2GlobalCoord(laser_point, map_laser_point);  // x' = R*x + t
    estimated_pose.RotateNormalVector(laser_point, map_laser_point);  // n' = R*n

    global_laser_points.emplace_back(map_laser_point);
  }

  point_cloud_map_ptr_->AddPose(estimated_pose);
  point_cloud_map_ptr_->AddLaserPoints(global_laser_points);
  point_cloud_map_ptr_->SetLastPose(estimated_pose);
  point_cloud_map_ptr_->SetLastScan(curr_scan);
  point_cloud_map_ptr_->MakeLocalMap();

  return;
}

}  // namespace sample_slam