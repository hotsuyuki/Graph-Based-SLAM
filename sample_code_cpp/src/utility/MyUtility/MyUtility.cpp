#include "MyUtility.h"

#include <Eigen/SVD>


namespace sample_slam {

double MyUtility::CastRadian(double radian) {
  while (radian < -M_PI) {
    radian += 2.0*M_PI;
  }
  while (M_PI < radian) {
    radian -= 2.0*M_PI;
  }

  return radian;
}


double MyUtility::AddRadian(double radian1, double radian2) {
  double radian = radian1 + radian2;
  radian = CastRadian(radian);
  
  return radian;
}


double MyUtility::SubtractRadian(double radian1, double radian2) {
  double radian = radian1 - radian2;
  radian = CastRadian(radian);
  
  return radian;
}


void MyUtility::InverseMatrixSVD(const Eigen::Matrix3f& matrix, Eigen::Matrix3f& inverse_matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXf> jacobi_svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXf matrix_u = jacobi_svd.matrixU();
  Eigen::MatrixXf matrix_v = jacobi_svd.matrixV();
  Eigen::VectorXf singular_values = jacobi_svd.singularValues();

  std::size_t dimension = 3;

  Eigen::Matrix3f M1;
  for (std::size_t i = 0; i < dimension; ++i) {
    for (std::size_t j = 0; j < dimension; ++j) {
      M1(i, j) = matrix_u(j, i) / singular_values(i);
    }
  }

  for (std::size_t i = 0; i < dimension; ++i) {
    for (std::size_t j = 0; j < dimension; ++j) {
      inverse_matrix(i, j) = 0.0;
      for (std::size_t k = 0; k < dimension; ++k) {
        inverse_matrix(i, j) += matrix_v(i, k) * M1(k, j);
      }
    }
  }

  return;
}

}  // namespace sample_slam