#include "LaserPoint2D.h"

#include <cmath>
#include <iostream>
// #include <vector>

// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include "gtest/gtest.h"
#include "MyUtility.h"


namespace sample_slam {

#define DEBUG_PRINT(var) std::cout << #var << " = " << var << "\n";


class LaserPoint2DTest : public ::testing::Test {
 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  double max_range_ = 2.0;  // [m]
  double delta_range_ = 1.0;  // [m]
};


TEST_F(LaserPoint2DTest, RangeYaw2XYTest) {
  LaserPoint2D laser_point;
  for (double range = -max_range_; range <= max_range_; range += delta_range_) {
    laser_point.RangeYaw2XY(range, DEG2RAD(-180.0));
    EXPECT_DOUBLE_EQ(-range, laser_point.x);
    EXPECT_NEAR(0.0, laser_point.y, 1.0e-6);

    laser_point.RangeYaw2XY(range, DEG2RAD(-150.0));
    EXPECT_DOUBLE_EQ(-range * (sqrt(3.0) / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range * (1.0 / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-135.0));
    EXPECT_DOUBLE_EQ(-range / sqrt(2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range / sqrt(2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-120.0));
    EXPECT_DOUBLE_EQ(-range * (1.0 / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range * (sqrt(3.0) / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-90.0));
    EXPECT_NEAR(0.0, laser_point.x, 1.0e-6);
    EXPECT_DOUBLE_EQ(-range, laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-60.0));
    EXPECT_DOUBLE_EQ(range * (1.0 / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range * (sqrt(3.0) / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-45.0));
    EXPECT_DOUBLE_EQ(range / sqrt(2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range / sqrt(2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(-30.0));
    EXPECT_DOUBLE_EQ(range * (sqrt(3.0) / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(-range * (1.0 / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(0.0));
    EXPECT_DOUBLE_EQ(range, laser_point.x);
    EXPECT_NEAR(0.0, laser_point.y, 1.0e-6);

    laser_point.RangeYaw2XY(range, DEG2RAD(30.0));
    EXPECT_DOUBLE_EQ(range * (sqrt(3.0) / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range * (1.0 / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(45.0));
    EXPECT_DOUBLE_EQ(range / sqrt(2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range / sqrt(2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(60.0));
    EXPECT_DOUBLE_EQ(range * (1.0 / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range * (sqrt(3.0) / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(90.0));
    EXPECT_NEAR(0.0, laser_point.x, 1.0e-6);
    EXPECT_DOUBLE_EQ(range, laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(120.0));
    EXPECT_DOUBLE_EQ(-range * (1.0 / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range * (sqrt(3.0) / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(135.0));
    EXPECT_DOUBLE_EQ(-range / sqrt(2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range / sqrt(2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(150.0));
    EXPECT_DOUBLE_EQ(-range * (sqrt(3.0) / 2.0), laser_point.x);
    EXPECT_DOUBLE_EQ(range * (1.0 / 2.0), laser_point.y);

    laser_point.RangeYaw2XY(range, DEG2RAD(180.0));
    EXPECT_DOUBLE_EQ(-range, laser_point.x);
    EXPECT_NEAR(0.0, laser_point.y, 1.0e-6);
  }
}


}  // namespace sample_slam