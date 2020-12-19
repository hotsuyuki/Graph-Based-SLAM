#ifndef UTILITY_POSE2D_H_
#define UTILITY_POSE2D_H_


#include <cmath>

// utility/
#include "LaserPoint2D.h"
#include "MyUtility.h"


namespace sample_slam {

struct Pose2D {
  double x, y, yaw;  // [m], [m], [rad]
  double rotation_matrix[2][2];

  Pose2D() : x(0.0), y(0.0), yaw(0.0) {
    rotation_matrix[0][0] = 1.0;  rotation_matrix[0][1] = 0.0;
    rotation_matrix[1][0] = 0.0;  rotation_matrix[1][1] = 1.0;
  }

  Pose2D(double _x, double _y, double _yaw) : x(_x), y(_y), yaw(MyUtility::CastRadian(_yaw)) {
    CalcRotationMatrix();
  }

  Pose2D(const Pose2D& other) : x(other.x), y(other.y), yaw(MyUtility::CastRadian(other.yaw)) {
    CalcRotationMatrix();
  }

  void CalcRotationMatrix() {
    yaw = MyUtility::CastRadian(yaw);
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    rotation_matrix[0][0] = cos_yaw;  rotation_matrix[0][1] = -sin_yaw;
    rotation_matrix[1][0] = sin_yaw;  rotation_matrix[1][1] = cos_yaw;
    return;
  }

  // x' = R*x + t
  void LocalCoord2GlobalCoord(const LaserPoint2D& local_laser_point, LaserPoint2D& global_laser_point) const {
    global_laser_point.x = rotation_matrix[0][0]*local_laser_point.x + rotation_matrix[0][1]*local_laser_point.y;
    global_laser_point.y = rotation_matrix[1][0]*local_laser_point.x + rotation_matrix[1][1]*local_laser_point.y;

    global_laser_point.x += x;
    global_laser_point.y += y;
    global_laser_point.id = local_laser_point.id;
    return;
  }

  // x = R^{-1}*(x' - t)
  void GlobalCoord2LocalCoord(const LaserPoint2D& global_laser_point, LaserPoint2D& local_laser_point) const {
    double dx = global_laser_point.x - x;
    double dy = global_laser_point.y - y;

    double inverse_rotation_matrix[2][2];
    inverse_rotation_matrix[0][0] = rotation_matrix[0][0];  inverse_rotation_matrix[0][1] = rotation_matrix[1][0];  
    inverse_rotation_matrix[1][0] = rotation_matrix[0][1];  inverse_rotation_matrix[1][1] = rotation_matrix[1][1];  

    local_laser_point.x = inverse_rotation_matrix[0][0]*dx + inverse_rotation_matrix[0][1]*dy;
    local_laser_point.y = inverse_rotation_matrix[1][0]*dx + inverse_rotation_matrix[1][1]*dy;
    local_laser_point.id = global_laser_point.id;
    return;
  }

  // n' = R*n
  // local coord ---> global coord
  void RotateNormalVector(const LaserPoint2D& laser_point, LaserPoint2D& rotated_laser_point) const {
    rotated_laser_point.normal_x = rotation_matrix[0][0]*laser_point.normal_x + rotation_matrix[0][1]*laser_point.normal_y;
    rotated_laser_point.normal_y = rotation_matrix[1][0]*laser_point.normal_x + rotation_matrix[1][1]*laser_point.normal_y;
    return;
  }

  // n = R^{-1}*n'
  // global coord ---> local coord 
  void InverseRotateNormalVector(const LaserPoint2D& laser_point, LaserPoint2D& inverse_rotated_laser_point) const {
    double inverse_rotation_matrix[2][2];
    inverse_rotation_matrix[0][0] = rotation_matrix[0][0];  inverse_rotation_matrix[0][1] = rotation_matrix[1][0];  
    inverse_rotation_matrix[1][0] = rotation_matrix[0][1];  inverse_rotation_matrix[1][1] = rotation_matrix[1][1];  

    inverse_rotated_laser_point.normal_x = inverse_rotation_matrix[0][0]*laser_point.normal_x + inverse_rotation_matrix[0][1]*laser_point.normal_y;
    inverse_rotated_laser_point.normal_y = inverse_rotation_matrix[1][0]*laser_point.normal_x + inverse_rotation_matrix[1][1]*laser_point.normal_y;
    return;
  }
 
  // Compounding operator (+)
  Pose2D operator+(const Pose2D& addition_relative) const {
    Pose2D result_absolute;
    result_absolute.x = x + rotation_matrix[0][0]*addition_relative.x + rotation_matrix[0][1]*addition_relative.y;
    result_absolute.y = y + rotation_matrix[1][0]*addition_relative.x + rotation_matrix[1][1]*addition_relative.y;
    result_absolute.yaw = MyUtility::AddRadian(yaw, addition_relative.yaw);
    result_absolute.CalcRotationMatrix();

    return result_absolute;
  }

  // Inverse compounding operator (-)
  // http://mrpt.ual.es/reference/0.9.1/classmrpt_1_1poses_1_1_c_pose.html#a007ab7e1ee2b722733f1bd52e6c89b3a
  Pose2D operator-(const Pose2D& subtraction_absolute) const {
    double dx = x - subtraction_absolute.x;
    double dy = y - subtraction_absolute.y;

    double inverse_rotation_matrix[2][2];
    inverse_rotation_matrix[0][0] = subtraction_absolute.rotation_matrix[0][0];  inverse_rotation_matrix[0][1] = subtraction_absolute.rotation_matrix[1][0];  
    inverse_rotation_matrix[1][0] = subtraction_absolute.rotation_matrix[0][1];  inverse_rotation_matrix[1][1] = subtraction_absolute.rotation_matrix[1][1];  

    Pose2D result_relative;
    result_relative.x = inverse_rotation_matrix[0][0]*dx + inverse_rotation_matrix[0][1]*dy;
    result_relative.y = inverse_rotation_matrix[1][0]*dx + inverse_rotation_matrix[1][1]*dy;
    result_relative.yaw = MyUtility::SubtractRadian(yaw, subtraction_absolute.yaw);
    result_relative.CalcRotationMatrix();

    return result_relative;
  }
};

}  // namespace sample_slam


#endif  // UTILITY_POSE2D_H_