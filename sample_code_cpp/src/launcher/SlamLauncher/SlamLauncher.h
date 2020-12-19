#ifndef LAUNCHER_SLAMLAUNCHER_H_
#define LAUNCHER_SLAMLAUNCHER_H_


#include <unistd.h>
#include <string>

// launcher/
#include "MapDrawer.h"
#include "SensorDataReader.h"
// slam/
#include "PointCloudMap.h"
#include "SlamFrontend.h"
// utility/
#include "Pose2D.h"
#include "Scan2D.h"


#define VIZ_MODE_SCAN "scan"
#define VIZ_MODE_ODOMETRY "odom"
#define VIZ_MODE_SLAM "slam"


namespace sample_slam {

class SlamLauncher {
 public:
  SlamLauncher()
    : num_skip_draw_(10) {}

  ~SlamLauncher() {}

  void SetPointCloudMapPtr(PointCloudMap* point_cloud_map_ptr) {
    point_cloud_map_ptr_ = point_cloud_map_ptr;
  }

  void SetIsScanPreprocess(bool is_scan_preprocess) {
    is_scan_preprocess_ = is_scan_preprocess;
  }

  SlamFrontend* GetSlamFrontendPtr() {
    return &slam_frontend_;
  }

  bool OpenSensorDataFile(std::string sensor_data_filename);
  void VisualizeScan();
  void Run(std::string viz_option);


 private:
  void RunOdometry(const Scan2D& scan);
  void RunSlam(const Scan2D& scan, int count);

  PointCloudMap* point_cloud_map_ptr_;  // Pointer to abstract class, which is customizable
  bool is_scan_preprocess_;

  SensorDataReader sensor_data_reader_;
  Pose2D init_pose_;
  SlamFrontend slam_frontend_;
  MapDrawer map_drawer_;
  int num_skip_draw_;
};

}  // namespace sample_slam


#endif  // LAUNCHER_SLAMLAUNCHER_H_