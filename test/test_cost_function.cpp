// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <nobleo_odometry_calibration/cost_function.hpp>

using nobleo_odometry_calibration::CostFunctor;
using nobleo_odometry_calibration::create_transform;

constexpr auto eps = 1e-15;

TEST(CostFunction, identity)
{
  auto odom_diff = create_transform(0, 0, 0);
  auto sensor_diff = create_transform(0, 0, 0);

  CostFunctor f{sensor_diff, odom_diff};

  std::array<double, 3> sensor_pose = {0, 0, 0};
  std::array<double, 3> residual = {0, 0, 0};
  f(&sensor_pose[0], &sensor_pose[1], &sensor_pose[2], residual.data());

  EXPECT_NEAR(residual[0], 0, eps);
  EXPECT_NEAR(residual[1], 0, eps);
  EXPECT_NEAR(residual[2], 0, eps);
}

TEST(CostFunction, rotated_sensor)
{
  auto odom_diff = create_transform(2, 0, 0);
  auto sensor_diff = create_transform(0, -2, 0);

  CostFunctor f{sensor_diff, odom_diff};

  std::array<double, 3> sensor_pose = {0, 0, M_PI_2};
  std::array<double, 3> residual = {0, 0, 0};
  f(&sensor_pose[0], &sensor_pose[1], &sensor_pose[2], residual.data());

  EXPECT_NEAR(residual[0], 0, eps);
  EXPECT_NEAR(residual[1], 0, eps);
  EXPECT_NEAR(residual[2], 0, eps);
}

TEST(CostFunction, offset_in_x)
{
  auto odom_diff = create_transform(0, 0, 0, 0, 0, M_PI_2);
  auto sensor_diff = create_transform(-0.2, 0.2, 0, 0, 0, M_PI_2);

  CostFunctor f{sensor_diff, odom_diff};

  std::array<double, 3> sensor_pose = {0.2, 0, 0};
  std::array<double, 3> residual = {0, 0, 0};
  f(&sensor_pose[0], &sensor_pose[1], &sensor_pose[2], residual.data());

  EXPECT_NEAR(residual[0], 0, eps);
  EXPECT_NEAR(residual[1], 0, eps);
  EXPECT_NEAR(residual[2], 0, eps);
}

TEST(CostFunction, offset_in_minus_x)
{
  auto odom_diff = create_transform(0, 0, 0, 0, 0, M_PI_2);
  auto sensor_diff = create_transform(0.2, -0.2, 0, 0, 0, M_PI_2);

  CostFunctor f{sensor_diff, odom_diff};

  std::array<double, 3> sensor_pose = {-0.2, 0, 0};
  std::array<double, 3> residual = {0, 0, 0};
  f(&sensor_pose[0], &sensor_pose[1], &sensor_pose[2], residual.data());

  EXPECT_NEAR(residual[0], 0, eps);
  EXPECT_NEAR(residual[1], 0, eps);
  EXPECT_NEAR(residual[2], 0, eps);
}

TEST(CostFunction, offset_in_y)
{
  auto odom_diff = create_transform(0, 0, 0, 0, 0, M_PI_2);
  auto sensor_diff = create_transform(-0.2, -0.2, 0, 0, 0, M_PI_2);

  CostFunctor f{sensor_diff, odom_diff};

  std::array<double, 3> sensor_pose = {0, 0.2, 0};
  std::array<double, 3> residual = {0, 0, 0};
  f(&sensor_pose[0], &sensor_pose[1], &sensor_pose[2], residual.data());

  EXPECT_NEAR(residual[0], 0, eps);
  EXPECT_NEAR(residual[1], 0, eps);
  EXPECT_NEAR(residual[2], 0, eps);
}
