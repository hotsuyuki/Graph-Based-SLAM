#include "MyUtility.h"

#include <cmath>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gtest/gtest.h"


namespace sample_slam {

#define DEBUG_PRINT(var) std::cout << #var << " = " << var << "\n";


class MyUtilityTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}
};


TEST_F(MyUtilityTest, CastRadianTest) {
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::CastRadian(-4.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::CastRadian(-3.5*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::CastRadian(-3.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::CastRadian(-2.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::CastRadian(-2.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::CastRadian(-1.5*M_PI));

  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::CastRadian(-1.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::CastRadian(-0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::CastRadian(0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::CastRadian(0.5*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::CastRadian(1.0*M_PI));

  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::CastRadian(1.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::CastRadian(2.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::CastRadian(2.5*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::CastRadian(3.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::CastRadian(3.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::CastRadian(4.0*M_PI));
}


TEST_F(MyUtilityTest, AddRadianTest) {
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(-1.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(-1.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::AddRadian(-1.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(-1.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(-1.0*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(-0.5*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::AddRadian(-0.5*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(-0.5*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(-0.5*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(-0.5*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::AddRadian(0.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(0.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(0.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(0.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::AddRadian(0.0*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(0.5*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(0.5*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(0.5*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::AddRadian(0.5*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(0.5*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(1.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::AddRadian(1.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::AddRadian(1.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::AddRadian(1.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::AddRadian(1.0*M_PI, 1.0*M_PI));
}


TEST_F(MyUtilityTest, SubtractRadianTest) {
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(-1.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(-1.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::SubtractRadian(-1.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(-1.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(-1.0*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(-0.5*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(-0.5*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(-0.5*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::SubtractRadian(-0.5*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(-0.5*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::SubtractRadian(0.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(0.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(0.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(0.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(-1.0*M_PI, MyUtility::SubtractRadian(0.0*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(0.5*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::SubtractRadian(0.5*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(0.5*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(0.5*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(0.5*M_PI, 1.0*M_PI));

  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(1.0*M_PI, -1.0*M_PI));
  EXPECT_DOUBLE_EQ(-0.5*M_PI, MyUtility::SubtractRadian(1.0*M_PI, -0.5*M_PI));
  EXPECT_DOUBLE_EQ(1.0*M_PI, MyUtility::SubtractRadian(1.0*M_PI, 0.0*M_PI));
  EXPECT_DOUBLE_EQ(0.5*M_PI, MyUtility::SubtractRadian(1.0*M_PI, 0.5*M_PI));
  EXPECT_DOUBLE_EQ(0.0*M_PI, MyUtility::SubtractRadian(1.0*M_PI, 1.0*M_PI));
}


TEST_F(MyUtilityTest, InverseMatrixSVDTest) {
  Eigen::Matrix3f indentity_matrix = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f expect_inverse_indentity_matrix = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f inverse_indentity_matrix;
  MyUtility::InverseMatrixSVD(indentity_matrix, inverse_indentity_matrix);
  EXPECT_TRUE(expect_inverse_indentity_matrix.isApprox(inverse_indentity_matrix));

  double deg2rad_45 = DEG2RAD(45.0);
  Eigen::Matrix3f rotation_matrix_2d;
  rotation_matrix_2d << cos(deg2rad_45), -sin(deg2rad_45), 0.0,
                        sin(deg2rad_45),  cos(deg2rad_45), 0.0,
                                    0.0,              0.0, 1.0;
  Eigen::Matrix3f expect_inverse_rotation_matrix_2d = rotation_matrix_2d.transpose();
  Eigen::Matrix3f inverse_rotation_matrix_2d;
  MyUtility::InverseMatrixSVD(rotation_matrix_2d, inverse_rotation_matrix_2d);
  EXPECT_TRUE(expect_inverse_rotation_matrix_2d.isApprox(inverse_rotation_matrix_2d));

  double deg2rad_30 = DEG2RAD(30.0);
  Eigen::Matrix3f rotation_matrix_yaw;
  rotation_matrix_yaw = Eigen::AngleAxisf(deg2rad_30, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotation_matrix_pitch;
  rotation_matrix_pitch = Eigen::AngleAxisf(deg2rad_30, Eigen::Vector3f::UnitY());
  Eigen::Matrix3f rotation_matrix_roll;
  rotation_matrix_roll = Eigen::AngleAxisf(deg2rad_30, Eigen::Vector3f::UnitX());
  Eigen::Matrix3f rotation_matrix_3d = rotation_matrix_yaw
                                         * rotation_matrix_pitch
                                           * rotation_matrix_roll;
  Eigen::Matrix3f expect_inverse_rotation_matrix_3d = rotation_matrix_roll.transpose()
                                                        * rotation_matrix_pitch.transpose()
                                                          * rotation_matrix_yaw.transpose();
  Eigen::Matrix3f inverse_rotation_matrix_3d;
  MyUtility::InverseMatrixSVD(rotation_matrix_3d, inverse_rotation_matrix_3d);
  EXPECT_TRUE(expect_inverse_rotation_matrix_3d.isApprox(inverse_rotation_matrix_3d));
}

}  // namespace sample_slam