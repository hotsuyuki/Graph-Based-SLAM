#include "SlamLauncher.h"

#include <iostream>

#include "ScanPreprocessor.h"


namespace sample_slam {

bool SlamLauncher::OpenSensorDataFile(std::string sensor_data_filename) {
  return sensor_data_reader_.OpenSensorDataFile(sensor_data_filename);
}


void SlamLauncher::VisualizeScan() {
  map_drawer_.SetRange(Scan2D::MAX_SCAN_RANGE);

  std::size_t count = 0;
  bool is_eof = false;

  while (!is_eof) {
    std::cout << "[SlamLauncher::VisualizeScan()] count = " << count << "\n";

    Scan2D scan;
    is_eof = sensor_data_reader_.LoadScan(count, scan);

    if (is_scan_preprocess_) {
      ScanPreprocessor scan_preprocessor;
      scan_preprocessor.ResampleLaserPoints(scan.laser_points);
    }

    map_drawer_.DrawScanGnuplot(scan);
    usleep(0.1 * 1000 * 1000);  // [us]

    ++count;
  }

  sensor_data_reader_.CloseSensorDataFile();

  return;
}


void SlamLauncher::Run(std::string viz_option) {
  std::size_t count = 0;
  bool is_eof = false;

  while (!is_eof) {
    std::cout << "[SlamLauncher::Run()] count = " << count << "\n";

    Scan2D scan;
    is_eof = sensor_data_reader_.LoadScan(count, scan);

    if (count == 0) {
      init_pose_ = scan.pose;
      init_pose_.CalcRotationMatrix();
    }

    if (viz_option == VIZ_MODE_ODOMETRY) {
      RunOdometry(scan);
    } else if (viz_option == VIZ_MODE_SLAM) {
      RunSlam(scan, count);
    } else {
      std::cout << "[SlamLauncher::Run()] ";
      std::cerr << "Error: viz_option = " << viz_option << "\n";
    }

    if (count % num_skip_draw_ == 0) {
      map_drawer_.DrawMapGnuplot(*point_cloud_map_ptr_);
    }

    ++count;
  }

  sensor_data_reader_.CloseSensorDataFile();

  // Endless loop for remaining the map drawing after processing.
  while (true) {
    usleep(0.1 * 1000 * 1000);  // [us]
    std::cout << "[SlamLauncher::VisualizeSlam()] EOF: Ctrl-c to end... \n";
  }
}


void SlamLauncher::RunOdometry(const Scan2D& scan) {
  Pose2D odometry_pose = scan.pose - init_pose_;

  std::vector<LaserPoint2D> global_laser_points;
  for (std::size_t i = 0; i < scan.laser_points.size(); ++i) {
    LaserPoint2D global_laser_point;
    odometry_pose.LocalCoord2GlobalCoord(scan.laser_points[i], global_laser_point);
    global_laser_points.emplace_back(global_laser_point);
  }

  point_cloud_map_ptr_->AddPose(odometry_pose);
  point_cloud_map_ptr_->AddLaserPoints(global_laser_points);
  point_cloud_map_ptr_->MakeGlobalMap();

  std::cout << "[SlamLauncher::RunOdometry()] "
            << "odometry_pose.x = " << odometry_pose.x << ", odometry_pose.y = " << odometry_pose.y
            << ", odometry_pose.yaw = " << odometry_pose.yaw <<"\n";

  return;
}


void SlamLauncher::RunSlam(const Scan2D& scan, int count) {
  slam_frontend_.RunSlam(scan, count);
  
  return;
}

}  // namespace sample_slam