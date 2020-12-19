#ifndef UTILITY_GRIDTABLE2D_H_
#define UTILITY_GRIDTABLE2D_H_


#include <cmath>
#include <iostream>
#include <vector>

// utility/
#include "LaserPoint2D.h"
#include "Pose2D.h"


namespace sample_slam {

struct GridTable2D {
  static constexpr double HALF_TABLE_LENGTH = 40.0;  // [m]
  static constexpr double CELL_RESOLUTION = 0.05;  // [m/px]
  static constexpr double NEAREST_RANGE = 0.2;  // [m]

  int half_num_cell;  // [px]
  int num_cell;  // [px]
  std::vector<std::vector<const LaserPoint2D*>> table;

  GridTable2D() {
    half_num_cell = static_cast<int>(HALF_TABLE_LENGTH / CELL_RESOLUTION);
    num_cell = 2 * half_num_cell + 1;
    table.resize(num_cell * num_cell);
    clear();
  }

  ~GridTable2D() {}

  void clear() {
    for (std::size_t idx = 0; idx < table.size(); ++idx) {
      if (0 < table[idx].size()) {
        table[idx].clear();
      }
    }

    return;
  }

  void AddLaserPoint(const LaserPoint2D* laser_point_ptr) {
    int x_i = static_cast<int>(laser_point_ptr->x / CELL_RESOLUTION) + half_num_cell;
    if (x_i < 0 || num_cell <= x_i) {
      return;
    }

    int y_i = static_cast<int>(laser_point_ptr->y / CELL_RESOLUTION) + half_num_cell;
    if (y_i < 0 || num_cell <= y_i) {
      return;
    }

    int idx = y_i * num_cell + x_i;
    table[idx].emplace_back(laser_point_ptr);

    return;
  }

  void AddLaserPoint(const LaserPoint2D& laser_point) {
    int x_i = static_cast<int>(laser_point.x / CELL_RESOLUTION) + half_num_cell;
    if (x_i < 0 || num_cell <= x_i) {
      return;
    }

    int y_i = static_cast<int>(laser_point.y / CELL_RESOLUTION) + half_num_cell;
    if (y_i < 0 || num_cell <= y_i) {
      return;
    }

    int idx = y_i * num_cell + x_i;
    table[idx].emplace_back(&laser_point);

    return;
  }

  void MakeCellPoints(int num_point_threshold, std::vector<LaserPoint2D>& map) {
    for (std::size_t idx = 0; idx < table.size(); ++idx) {
      if (table[idx].size() < num_point_threshold) {
        continue;
      }

      int latest_id = -1;
      double centroid_x = 0.0;
      double centroid_y = 0.0;
      double mean_normal_x = 0.0;
      double mean_normal_y = 0.0;

      for (std::size_t i = 0; i < table[idx].size(); ++i) {
        if (latest_id < table[idx][i]->id) {
          latest_id = table[idx][i]->id;
        }
        centroid_x += table[idx][i]->x;
        centroid_y += table[idx][i]->y;
        mean_normal_x += table[idx][i]->normal_x;
        mean_normal_y += table[idx][i]->normal_y;
      }

      centroid_x /= table[idx].size();
      centroid_y /= table[idx].size();
      mean_normal_x /= hypot(mean_normal_x, mean_normal_y);
      mean_normal_y /= hypot(mean_normal_x, mean_normal_y);

      LaserPoint2D cell_laser_point;
      cell_laser_point.id = latest_id;
      cell_laser_point.x = centroid_x;
      cell_laser_point.y = centroid_y;
      cell_laser_point.normal_x = mean_normal_x;
      cell_laser_point.normal_y = mean_normal_y;
      cell_laser_point.point_type = PointType::LINE;

      map.emplace_back(cell_laser_point);
    }

    return;
  }

  const LaserPoint2D* FindNearestLaserPoint(const LaserPoint2D& curr_laser_point){
    int curr_x_i = static_cast<int>(curr_laser_point.x / CELL_RESOLUTION) + half_num_cell;
    if (curr_x_i < 0 || num_cell <= curr_x_i) {
      return nullptr;
    }

    int curr_y_i = static_cast<int>(curr_laser_point.y / CELL_RESOLUTION) + half_num_cell;
    if (curr_y_i < 0 || num_cell <= curr_y_i) {
      return nullptr;
    } 

    const LaserPoint2D* nearest_laser_point_ptr = nullptr;
    double nearest_distance = __DBL_MAX__;  // [m]
    int range = static_cast<int>(NEAREST_RANGE / CELL_RESOLUTION);  // [px]

    for (int ny = -range; ny <= range; ++ny) {
      int y_i = curr_y_i + ny;
      if (y_i < 0 || num_cell <= y_i) {
        continue;
      }

      for (int nx = -range; nx <= range; ++nx) {
        int x_i = curr_x_i + nx;
        if (x_i < 0 || num_cell <= x_i) {
          continue;
        }

        int idx = y_i * num_cell + x_i;
        for(std::size_t i = 0; i < table[idx].size(); ++i){
          const LaserPoint2D* reference_laser_point_ptr = table[idx][i];

          double dx = reference_laser_point_ptr->x - curr_laser_point.x;
          double dy = reference_laser_point_ptr->y - curr_laser_point.y;
          double distance = hypot(dx, dy);

          if (distance < nearest_distance && distance < NEAREST_RANGE) {
            nearest_laser_point_ptr = reference_laser_point_ptr;
            nearest_distance = distance;
          }
        }
      }
    }

    return nearest_laser_point_ptr;
  }
};

}  // namespace sample_slam


#endif  // UTILITY_LASERPOINT2D_H_