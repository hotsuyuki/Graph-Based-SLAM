#include "Pose2D.h"

#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gtest/gtest.h"
#include "LaserPoint2D.h"
#include "MyUtility.h"


namespace sample_slam {

#define DEBUG_PRINT(var) std::cout << #var << " = " << var << "\n";


class Pose2DTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  double max_distance_ = 2.0;  // [m]
  double delta_distance_ = 1.0;  // [m]

  double max_angle_ = M_PI;  // [rad]
  double delta_angle_ = 0.5*M_PI;  // [rad]
};


TEST_F(Pose2DTest, CalcRotationMatrixTest) {
  for (double yaw = -max_angle_; yaw <= max_angle_; yaw += delta_angle_) {
    Pose2D pose;
    pose.yaw = yaw;  // [rad]
    pose.CalcRotationMatrix();

    Eigen::Matrix2f eigen_rotation_matrix;
    eigen_rotation_matrix = Eigen::Rotation2Df(yaw);

    EXPECT_NEAR(eigen_rotation_matrix(0,0), pose.rotation_matrix[0][0], 1.0e-6);
    EXPECT_NEAR(eigen_rotation_matrix(0,1), pose.rotation_matrix[0][1], 1.0e-6);
    EXPECT_NEAR(eigen_rotation_matrix(1,0), pose.rotation_matrix[1][0], 1.0e-6);
    EXPECT_NEAR(eigen_rotation_matrix(1,1), pose.rotation_matrix[1][1], 1.0e-6);
  }
}


TEST_F(Pose2DTest, LocalCoord2GlobalCoordTest) {
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);

        Eigen::Affine2f eigen_affine_matrix = Eigen::Translation2f(pose_x, pose_y)
                                                * Eigen::Rotation2Df(pose_yaw);
        
        for (double local_laser_point_yaw = -max_angle_; local_laser_point_yaw <= max_angle_; local_laser_point_yaw += delta_angle_) {
          LaserPoint2D local_laser_point;
          local_laser_point.x = cos(local_laser_point_yaw);
          local_laser_point.y = sin(local_laser_point_yaw);
          LaserPoint2D global_laser_point;
          pose.LocalCoord2GlobalCoord(local_laser_point, global_laser_point);

          Eigen::Vector2f eigen_local_laser_point;
          eigen_local_laser_point << local_laser_point.x,
                                     local_laser_point.y;
          Eigen::Vector2f eigen_global_laser_point = eigen_affine_matrix * eigen_local_laser_point;

          EXPECT_NEAR(eigen_global_laser_point(0), global_laser_point.x, 1.0e-6);
          EXPECT_NEAR(eigen_global_laser_point(1), global_laser_point.y, 1.0e-6);
        }
      }
    }
  }
}


TEST_F(Pose2DTest, GlobalCoord2LocalCoordTest) {
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);

        Eigen::Affine2f eigen_affine_matrix = Eigen::Translation2f(pose_x, pose_y)
                                                * Eigen::Rotation2Df(pose_yaw);
        
        for (double local_laser_point_yaw = -max_angle_; local_laser_point_yaw <= max_angle_; local_laser_point_yaw += delta_angle_) {
          LaserPoint2D expect_local_laser_point;
          expect_local_laser_point.x = cos(local_laser_point_yaw);
          expect_local_laser_point.y = sin(local_laser_point_yaw);

          Eigen::Vector2f eigen_local_laser_point;
          eigen_local_laser_point << expect_local_laser_point.x,
                                     expect_local_laser_point.y;
          Eigen::Vector2f eigen_global_laser_point = eigen_affine_matrix * eigen_local_laser_point;

          LaserPoint2D global_laser_point;
          global_laser_point.x = eigen_global_laser_point(0);
          global_laser_point.y = eigen_global_laser_point(1);

          LaserPoint2D actual_local_laser_point;
          pose.GlobalCoord2LocalCoord(global_laser_point, actual_local_laser_point);

          EXPECT_NEAR(expect_local_laser_point.x, actual_local_laser_point.x, 1.0e-6);
          EXPECT_NEAR(expect_local_laser_point.y, actual_local_laser_point.y, 1.0e-6);
        }
      }
    }
  }
}


TEST_F(Pose2DTest, RotateNormalVectorTest) {
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);

        Eigen::Affine2f eigen_rotation_matrix;
        eigen_rotation_matrix = Eigen::Rotation2Df(pose_yaw);

        for (double local_laser_point_yaw = -max_angle_; local_laser_point_yaw <= max_angle_; local_laser_point_yaw += delta_angle_) {
          LaserPoint2D local_laser_point;
          local_laser_point.x = cos(local_laser_point_yaw);
          local_laser_point.y = sin(local_laser_point_yaw);
          local_laser_point.normal_x = cos(local_laser_point_yaw + 0.5*M_PI);
          local_laser_point.normal_y = sin(local_laser_point_yaw + 0.5*M_PI);

          LaserPoint2D global_laser_point;
          pose.RotateNormalVector(local_laser_point, global_laser_point);

          Eigen::Vector2f eigen_local_laser_point_normal_vector;
          eigen_local_laser_point_normal_vector << local_laser_point.normal_x,
                                                   local_laser_point.normal_y;
          Eigen::Vector2f eigen_global_laser_point_normal_vector = eigen_rotation_matrix
                                                                     * eigen_local_laser_point_normal_vector;

          EXPECT_NEAR(eigen_global_laser_point_normal_vector(0), global_laser_point.normal_x, 1.0e-6);
          EXPECT_NEAR(eigen_global_laser_point_normal_vector(1), global_laser_point.normal_y, 1.0e-6);
        }
      }
    }
  }
}


TEST_F(Pose2DTest, InverseRotateNormalVectorTest) {
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);

        Eigen::Affine2f eigen_rotation_matrix;
        eigen_rotation_matrix = Eigen::Rotation2Df(pose_yaw);
 
        for (double local_laser_point_yaw = -max_angle_; local_laser_point_yaw <= max_angle_; local_laser_point_yaw += delta_angle_) {
          LaserPoint2D expect_local_laser_point;
          expect_local_laser_point.x = cos(local_laser_point_yaw);
          expect_local_laser_point.y = sin(local_laser_point_yaw);
          expect_local_laser_point.normal_x = cos(local_laser_point_yaw + 0.5*M_PI);
          expect_local_laser_point.normal_y = sin(local_laser_point_yaw + 0.5*M_PI);

          Eigen::Vector2f eigen_local_laser_point_normal_vector;
          eigen_local_laser_point_normal_vector << expect_local_laser_point.normal_x,
                                                   expect_local_laser_point.normal_y;
          Eigen::Vector2f eigen_global_laser_point_normal_vector = eigen_rotation_matrix
                                                                     * eigen_local_laser_point_normal_vector;

          LaserPoint2D global_laser_point;
          global_laser_point.normal_x = eigen_global_laser_point_normal_vector(0);
          global_laser_point.normal_y = eigen_global_laser_point_normal_vector(1);

          LaserPoint2D actual_local_laser_point;
          pose.InverseRotateNormalVector(global_laser_point, actual_local_laser_point);

          EXPECT_NEAR(expect_local_laser_point.normal_x, actual_local_laser_point.normal_x, 1.0e-6);
          EXPECT_NEAR(expect_local_laser_point.normal_y, actual_local_laser_point.normal_y, 1.0e-6);
        }
      }
    }
  }
}


TEST_F(Pose2DTest, OperatorPlusTest) {
  std::vector<Pose2D> absolute_poses;
  std::vector<Pose2D> addition_relative_poses;
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);
        absolute_poses.emplace_back(pose);
        addition_relative_poses.emplace_back(pose);
      }
    }
  }

  for (std::size_t i = 0; i < absolute_poses.size(); ++i) {
    const Pose2D& absolute_pose_i = absolute_poses[i];

    Eigen::Affine2f eigen_affine_matrix_i = Eigen::Translation2f(absolute_pose_i.x, absolute_pose_i.y)
                                              * Eigen::Rotation2Df(absolute_pose_i.yaw);
    
    for (std::size_t j = 0; j < addition_relative_poses.size(); ++j) {
      const Pose2D& addition_relative_pose_j = addition_relative_poses[j];
      Pose2D absolute_pose_i_plus_j = absolute_pose_i + addition_relative_pose_j;

      Eigen::Vector2f eigen_addition_relative_pose_j;
      eigen_addition_relative_pose_j << addition_relative_pose_j.x,
                                        addition_relative_pose_j.y;
      Eigen::Vector2f eigen_absolute_pose_i_plus_j = eigen_affine_matrix_i * eigen_addition_relative_pose_j;

      EXPECT_NEAR(eigen_absolute_pose_i_plus_j(0), absolute_pose_i_plus_j.x, 1.0e-6);
      EXPECT_NEAR(eigen_absolute_pose_i_plus_j(1), absolute_pose_i_plus_j.y, 1.0e-6);
    }
  }
}


TEST_F(Pose2DTest, OperatorMinusTest) {
  std::vector<Pose2D> absolute_poses;
  std::vector<Pose2D> subtraction_absolute_poses;
  for (double pose_x = -max_distance_; pose_x <= max_distance_; pose_x += delta_distance_) {
    for (double pose_y = -max_distance_; pose_y <= max_distance_; pose_y += delta_distance_) {
      for (double pose_yaw = -max_angle_; pose_yaw <= max_angle_; pose_yaw += delta_angle_) {
        Pose2D pose(pose_x, pose_y, pose_yaw);
        absolute_poses.emplace_back(pose);
        subtraction_absolute_poses.emplace_back(pose);
      }
    }
  }

  for (std::size_t i = 0; i < absolute_poses.size(); ++i) {
    const Pose2D& absolute_pose_i = absolute_poses[i];

    Eigen::Vector2f eigen_absolute_pose_i;
    eigen_absolute_pose_i << absolute_pose_i.x,
                             absolute_pose_i.y;
    
    for (std::size_t j = 0; j < subtraction_absolute_poses.size(); ++j) {
      const Pose2D& subtraction_absolute_pose_j = subtraction_absolute_poses[j];
      Pose2D relative_pose_i_minus_j = absolute_pose_i - subtraction_absolute_pose_j;

      Eigen::Affine2f eigen_inverse_translation_matrix_j;
      eigen_inverse_translation_matrix_j = Eigen::Translation2f(-subtraction_absolute_pose_j.x,
                                                                -subtraction_absolute_pose_j.y);

      Eigen::Affine2f eigen_inverse_rotation_matrix_j;
      eigen_inverse_rotation_matrix_j = Eigen::Rotation2Df(-subtraction_absolute_pose_j.yaw);

      Eigen::Vector2f eigen_relative_pose_i_minus_j =
        eigen_inverse_rotation_matrix_j * (eigen_inverse_translation_matrix_j * eigen_absolute_pose_i);

      EXPECT_NEAR(eigen_relative_pose_i_minus_j(0), relative_pose_i_minus_j.x, 1.0e-6);
      EXPECT_NEAR(eigen_relative_pose_i_minus_j(1), relative_pose_i_minus_j.y, 1.0e-6);
    }
  }
}

}  // namespace sample_slam