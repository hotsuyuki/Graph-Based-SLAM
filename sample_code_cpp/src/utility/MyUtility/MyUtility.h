#ifndef UTILITY_MYUTILITY_H_
#define UTILITY_MYUTILITY_H_


#include <cmath>

#include <Eigen/Core>


namespace sample_slam {

#define DEG2RAD(theta) (theta * M_PI / 180.0)
#define RAD2DEG(theta) (theta * 180.0 / M_PI)


class MyUtility {
 public:
  MyUtility() {}

  ~MyUtility() {}

  static double CastRadian(double radian);

  static double AddRadian(double radian1, double radian2);

  static double SubtractRadian(double radian1, double radian2);

  static void InverseMatrixSVD(const Eigen::Matrix3f& matrix, Eigen::Matrix3f& inverse_matrix);
};

}  // namespace sample_slam


#endif  // UTILITY_MYUTILITY_H_