#include "MapDrawer.h"

#include <iostream>


namespace sample_slam {

// Draw point cloud and robot trajectory
void MapDrawer::DrawMapGnuplot(const PointCloudMap& point_cloud_map) {
  const std::vector<LaserPoint2D>& global_map = point_cloud_map.GetGlobalMap();
  const std::vector<Pose2D>& poses = point_cloud_map.GetPoses();
  DrawGnuplot(global_map, poses);
  return;
}


// Draw only one scan
void MapDrawer::DrawScanGnuplot(const Scan2D& scan) {
  std::vector<Pose2D> poses{Pose2D(0.0, 0.0, 0.0)};
  DrawGnuplot(scan.laser_points, poses);
  return;
}


// Main drawing
void MapDrawer::DrawGnuplot(const std::vector<LaserPoint2D>& laser_points, const std::vector<Pose2D>& poses) {
  std::cout << "[MapDrawer::DrawGnuplot()] laser_points.size() = " << laser_points.size() << "\n";

  // Setting gnuplot
  fprintf(pipe_gnuplot_ptr_, "set multiplot \n");
  fprintf(pipe_gnuplot_ptr_, "plot '-' w p pt 7 ps 0.1 lc rgb 0x0, '-' with vector \n");

  // Draw point cloud
  for (std::size_t i = 0; i < laser_points.size(); i += num_skip_laser_point_) {
    fprintf(pipe_gnuplot_ptr_, "%lf %lf \n", laser_points[i].x, laser_points[i].y);
  }
  fprintf(pipe_gnuplot_ptr_, "e \n");

  // Draw robot trajectory
  for (std::size_t i = 0; i < poses.size(); i += num_skip_pose_) {
    double cos_yaw = cos(poses[i].yaw);
    double sin_yaw = sin(poses[i].yaw);

    double x_axis_x = axis_length_ * cos_yaw;
    double x_axis_y = axis_length_ * sin_yaw;
    fprintf(pipe_gnuplot_ptr_, "%lf %lf %lf %lf \n", poses[i].x, poses[i].y, x_axis_x, x_axis_y);

    double y_axis_x = axis_length_ * -sin_yaw;
    double y_axis_y = axis_length_ * cos_yaw;
    fprintf(pipe_gnuplot_ptr_, "%lf %lf %lf %lf \n", poses[i].x, poses[i].y, y_axis_x, y_axis_y);
  }
  fprintf(pipe_gnuplot_ptr_, "e \n");

  fflush(pipe_gnuplot_ptr_);

  return;
}

}  // namespace sample_slam