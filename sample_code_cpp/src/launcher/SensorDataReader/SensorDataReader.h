#ifndef LAUNCHER_SENSORDATAREADER_H_
#define LAUNCHER_SENSORDATAREADER_H_


#include <fstream>
#include <string>

// utility/
#include "MyUtility.h"
#include "Scan2D.h"


namespace sample_slam {

class SensorDataReader {
 public:
  SensorDataReader() : lidar_yaw_offset_(DEG2RAD(180.0)) {}

  ~SensorDataReader() {}

  bool OpenSensorDataFile(std::string sensor_data_filename);

  void CloseSensorDataFile();

  bool LoadScan(std::size_t count, Scan2D& scan);


 private:
  std::ifstream sensor_data_filestream_;
  double lidar_yaw_offset_;  // [rad]
};

}  // namespace sample_slam


#endif  // LAUNCHER_SENSORDATAREADER_H_