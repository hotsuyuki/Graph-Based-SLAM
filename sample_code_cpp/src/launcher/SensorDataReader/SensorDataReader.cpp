#include "SensorDataReader.h"

#include <iostream>
#include <vector>

// utility/
#include "LaserPoint2D.h"


namespace sample_slam {

#define LASERSCAN "LASERSCAN"


bool SensorDataReader::OpenSensorDataFile(std::string sensor_data_filename) {
  sensor_data_filestream_.open(sensor_data_filename);
  if (!sensor_data_filestream_.is_open()) {
    std::cerr << "[SensorDataReader::OpenSensorDataFile()] ";
    std::cerr << "Error: cannot open file " << sensor_data_filename << "\n\n";
    return false;
  }

  return true;
}


void SensorDataReader::CloseSensorDataFile() {
  sensor_data_filestream_.close();
  return;
}


bool SensorDataReader::LoadScan(std::size_t count, Scan2D& scan) {
  while(!sensor_data_filestream_.eof()) {
    std::string data_type;
    sensor_data_filestream_ >> data_type;

    // Loads and decodes the line if the data type is "LASERSCAN"
    if (data_type == LASERSCAN) {
      scan.id = count;

      /***********************************************************************************************************************************************************
      * LASERSCAN (scan_id) (second) (nano_second) (yaw_1 [deg]) (range_1 [m]) ... (yaw_n [deg]) (range_n [m]) (odom_x [m]) (odom_y [m]) (odom_yaw [rad])
      ***********************************************************************************************************************************************************/

      int scan_id, second, nano_second;
      sensor_data_filestream_ >> scan_id >> second >> nano_second;  // These are not used

      int num_laser_point;
      sensor_data_filestream_ >> num_laser_point;

      scan.laser_points.reserve(num_laser_point);
      for (std::size_t i = 0; i < num_laser_point; ++i) {
        double yaw, range;
        sensor_data_filestream_ >> yaw >> range;  // [deg], [m]

        if (range <= Scan2D::MIN_SCAN_RANGE || Scan2D::MAX_SCAN_RANGE <= range) {
          continue;
        }

        yaw = DEG2RAD(yaw) + lidar_yaw_offset_;  // [rad]

        LaserPoint2D laser_point;
        laser_point.id = count;
        laser_point.RangeYaw2XY(range, yaw);
        scan.laser_points.emplace_back(laser_point);
      }

      // Odometry data corresponding with the laser scan
      sensor_data_filestream_ >> scan.pose.x >> scan.pose.y >> scan.pose.yaw;  // [m], [m], [rad]
      scan.pose.CalcRotationMatrix();

      break;
    } 

    // Skips loading the line if the data type is NOT "LASERSCAN"
    std::string tmp;
    std::getline(sensor_data_filestream_, tmp);
  }

  return sensor_data_filestream_.eof();
}

}  // namespace sample_slam