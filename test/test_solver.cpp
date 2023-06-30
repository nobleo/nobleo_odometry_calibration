// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <nobleo_odometry_calibration/config.hpp>
#include <nobleo_odometry_calibration/solver.hpp>

using nobleo_odometry_calibration::create_transform;
using nobleo_odometry_calibration::OptimizeParameters;
using nobleo_odometry_calibration::Solver;

constexpr auto eps = 1e-8;

TEST(Solver, sensor_at_small_angle)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.theta = true;
    solver.configure(optimize_parameters);
  }
  auto p1 = create_transform(0, 0, 0, 0, 0, 0.2);
  auto p2 = create_transform(2, 0, 0, 0, 0, 0.2);
  auto p3 = create_transform(4, 0, 0, 0, 0, 0.2);
  auto sensor_diff = p1.inverse() * (p2);
  solver.add_constraint(sensor_diff, create_transform(2, 0, 0));
  sensor_diff = p2.inverse() * (p3);
  solver.add_constraint(sensor_diff, create_transform(2, 0, 0));
  EXPECT_TRUE(solver.solve());
  EXPECT_NEAR(solver.parameters().theta, 0.2, eps);
}

/**
 * @brief TEST
 *
 * The sensor is positioned at x=0.2. Test if the Solver can figure this out.
 */
TEST(Solver, sensor_offset_in_x)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.x = true;
    optimize_parameters.theta = true;
    solver.configure(optimize_parameters);
  }
  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(1.8, 0.2, 0, 0, 0, M_PI_2);  // turn 90 degrees
  auto sensor_diff = p1.inverse() * p2;
  solver.add_constraint(sensor_diff, create_transform(2, 0, 0));
  sensor_diff = p2.inverse() * p3;
  solver.add_constraint(sensor_diff, create_transform(0, 0, 0, 0, 0, M_PI_2));

  EXPECT_TRUE(solver.solve());
  EXPECT_NEAR(solver.parameters().x, 0.2, eps);
  EXPECT_NEAR(solver.parameters().y, 0, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);
}

/**
 * @brief TEST
 *
 * The sensor is positioned at x=0.2. Test if the Solver can figure this out while optimizing
 * x & y.
 */
TEST(Solver, sensor_offset_in_x_2)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.x = true;
    optimize_parameters.y = true;
    optimize_parameters.theta = true;
    solver.configure(optimize_parameters);
  }
  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(1.8, 0.2, 0, 0, 0, M_PI_2);  // turn 90 degrees
  auto sensor_diff = p1.inverse() * p2;
  solver.add_constraint(sensor_diff, create_transform(2, 0, 0));
  sensor_diff = p2.inverse() * p3;
  solver.add_constraint(sensor_diff, create_transform(0, 0, 0, 0, 0, M_PI_2));

  EXPECT_TRUE(solver.solve());
  EXPECT_NEAR(solver.parameters().x, 0.2, eps);
  EXPECT_NEAR(solver.parameters().y, 0, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);
}

/**
 * @brief TEST
 *
 * The sensor is positioned at y=0.2. Test if the Solver can figure this out.
 */
TEST(Solver, sensor_offset_in_y)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.x = true;
    optimize_parameters.y = true;
    optimize_parameters.theta = true;
    solver.configure(optimize_parameters);
  }

  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(2.2, -0.2, 0, 0, 0, -M_PI_2);  // turn 90 degrees
  auto sensor_diff = p1.inverse() * p2;
  solver.add_constraint(sensor_diff, create_transform(2, 0, 0));
  sensor_diff = p2.inverse() * p3;
  solver.add_constraint(sensor_diff, create_transform(0, 0, 0, 0, 0, -M_PI_2));

  EXPECT_TRUE(solver.solve());
  EXPECT_NEAR(solver.parameters().x, 0, eps);
  EXPECT_NEAR(solver.parameters().y, 0.2, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);
}

/**
 * @brief TEST
 *
 * The sensor measures 2m between poses, while odom measures 40% less.
 */
TEST(Solver, wheel_radius)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.wheel_radius_multiplier = true;
    solver.configure(optimize_parameters);
  }
  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(4, 0, 0);
  auto sensor_diff = p1.inverse() * (p2);
  solver.add_constraint(sensor_diff, create_transform(2 / 1.4, 0, 0));
  sensor_diff = p2.inverse() * (p3);
  solver.add_constraint(sensor_diff, create_transform(2 / 1.4, 0, 0));
  EXPECT_TRUE(solver.solve());
  EXPECT_NEAR(solver.parameters().wheel_radius_multiplier, 1.4, eps);
}

/**
 * @brief TEST
 *
 * The sensor measures 90 degrees between poses, while odom measures 40% less.
 */
TEST(Solver, wheel_separation)
{
  Solver solver;
  {
    OptimizeParameters optimize_parameters;
    optimize_parameters.wheel_separation_multiplier = true;
    solver.configure(optimize_parameters);
  }
  auto sensor_diff = create_transform(0, 0, 0, 0, 0, M_PI_2);
  solver.add_constraint(sensor_diff, create_transform(0, 0, 0, 0, 0, M_PI_2 / 1.4));
  sensor_diff = create_transform(0, 0, 0, 0, 0, M_PI_2);
  solver.add_constraint(sensor_diff, create_transform(0, 0, 0, 0, 0, M_PI_2 / 1.4));
  EXPECT_TRUE(solver.solve());
  // The separation is 40% too much, so the multiplier is the inverse
  EXPECT_NEAR(solver.parameters().wheel_separation_multiplier, 1 / 1.4, eps);
}
