#ifndef LAUNCHER_MAPDRAWER_H_
#define LAUNCHER_MAPDRAWER_H_


#include <stdio.h>
#include <vector>

// slam/
#include "PointCloudMap.h"
// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"
#include "Scan2D.h"


namespace sample_slam {

class MapDrawer {
 public:
  MapDrawer()
      : min_x_(-10.0), max_x_(10.0), min_y_(-10.0), max_y_(10.0), aspect_ratio_(-0.9),
        num_skip_laser_point_(1), num_skip_pose_(10),
        // num_skip_laser_point_(5), num_skip_pose_(10),
        axis_length_(0.5) {
    pipe_gnuplot_ptr_ = popen("gnuplot", "w");
    fprintf(pipe_gnuplot_ptr_, "set size ratio %lf \n", aspect_ratio_);
  }

  ~MapDrawer() {
    if (pipe_gnuplot_ptr_ != nullptr) {
      pclose(pipe_gnuplot_ptr_);
    }
  }

  void SetRange(double range) {
    min_x_ = -range;
    max_x_ = range;
    fprintf(pipe_gnuplot_ptr_, "set xrange [%lf:%lf] \n", min_x_, max_x_);

    min_y_ = -range;
    max_y_ = range;
    fprintf(pipe_gnuplot_ptr_, "set yrange [%lf:%lf] \n", min_y_, max_y_);
  }

  void DrawMapGnuplot(const PointCloudMap& point_cloud_map);
  void DrawScanGnuplot(const Scan2D& scan);
  void DrawGnuplot(const std::vector<LaserPoint2D>& laser_points, const std::vector<Pose2D>& poses);


 private:
  double min_x_, max_x_, min_y_, max_y_;
  double aspect_ratio_;

  int num_skip_laser_point_;
  int num_skip_pose_;

  double axis_length_;

  FILE* pipe_gnuplot_ptr_;
};

}  // namespace sample_slam


#endif  // LAUNCHER_MAPDRAWER_H_