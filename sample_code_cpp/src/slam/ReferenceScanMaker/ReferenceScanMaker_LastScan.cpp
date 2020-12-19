#include "ReferenceScanMaker_LastScan.h"


namespace sample_slam {

void ReferenceScanMaker_LastScan::MakeReferenceScan() {
  const Pose2D& last_pose = point_cloud_map_ptr_->GetLastPose();
  const Scan2D& last_scan = point_cloud_map_ptr_->GetLastScan();

  reference_scan_.laser_points.clear();
  for (std::size_t i = 0; i < last_scan.laser_points.size(); ++i) {
    const LaserPoint2D& laser_point = last_scan.laser_points[i];

    LaserPoint2D reference_laser_point;
    reference_laser_point.id = laser_point.id;
    reference_laser_point.point_type = laser_point.point_type;

    last_pose.LocalCoord2GlobalCoord(laser_point, reference_laser_point);  // x' = R*x + t
    last_pose.RotateNormalVector(laser_point, reference_laser_point);  // n' = R*n

    reference_scan_.laser_points.emplace_back(reference_laser_point);
  }

  return;
}

}  // namespace sample_slam