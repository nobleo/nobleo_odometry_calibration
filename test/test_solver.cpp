#include <gtest/gtest.h>
#include <ros/console.h>

#include "../src/solver.hpp"

using nobleo_gps_calibration::create_transform;
using nobleo_gps_calibration::Solver;

constexpr auto eps = 1e-9;

TEST(Solver, gps_at_small_angle)
{
  Solver solver;

  auto p1 = create_transform(0, 0, 0, 0, 0, 0.2);
  auto p2 = create_transform(2, 0, 0, 0, 0, 0.2);
  auto p3 = create_transform(4, 0, 0, 0, 0, 0.2);
  auto gps_diff = p1.inverse() * (p2);
  solver.add_constraint(gps_diff, create_transform(2, 0, 0));
  gps_diff = p2.inverse() * (p3);
  solver.add_constraint(gps_diff, create_transform(2, 0, 0));
  solver.solve();
  EXPECT_NEAR(solver.parameters().theta, 0.2, eps);
}

/**
 * @brief TEST
 *
 * The GPS receiver is positioned at x=0.2. Test if the Solver can figure this out.
 */
TEST(Solver, gps_offset_in_x)
{
  Solver solver;
  solver.configure({Solver::OptimizeParameters::XTHETA});

  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(1.8, 0.2, 0, 0, 0, M_PI_2);  // turn 90 degrees
  auto gps_diff = p1.inverse() * p2;
  solver.add_constraint(gps_diff, create_transform(2, 0, 0));
  gps_diff = p2.inverse() * p3;
  solver.add_constraint(gps_diff, create_transform(0, 0, 0, 0, 0, M_PI_2));

  solver.solve();
  EXPECT_NEAR(solver.parameters().x, 0.2, eps);
  EXPECT_NEAR(solver.parameters().y, 0, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);

  solver.configure({Solver::OptimizeParameters::XYTHETA});
  solver.solve();
  EXPECT_NEAR(solver.parameters().x, 0.2, eps);
  EXPECT_NEAR(solver.parameters().y, 0, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);
}

/**
 * @brief TEST
 *
 * The GPS receiver is positioned at y=0.2. Test if the Solver can figure this out.
 */
TEST(Solver, gps_offset_in_y)
{
  Solver solver;
  solver.configure({Solver::OptimizeParameters::XYTHETA});

  auto p1 = create_transform(0, 0, 0);
  auto p2 = create_transform(2, 0, 0);
  auto p3 = create_transform(2.2, -0.2, 0, 0, 0, -M_PI_2);  // turn 90 degrees
  auto gps_diff = p1.inverse() * p2;
  ROS_INFO_STREAM("gps_diff:\n" << gps_diff.matrix());
  solver.add_constraint(gps_diff, create_transform(2, 0, 0));
  gps_diff = p2.inverse() * p3;
  ROS_INFO_STREAM("gps_diff:\n" << gps_diff.matrix());
  solver.add_constraint(gps_diff, create_transform(0, 0, 0, 0, 0, -M_PI_2));

  solver.solve();
  EXPECT_NEAR(solver.parameters().x, 0, eps);
  EXPECT_NEAR(solver.parameters().y, 0.2, eps);
  EXPECT_NEAR(solver.parameters().theta, 0, eps);
}
