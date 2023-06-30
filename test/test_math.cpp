// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nobleo_odometry_calibration/math.hpp>

using nobleo_odometry_calibration::create_quaternion;
using nobleo_odometry_calibration::create_transform;
using nobleo_odometry_calibration::from_msg;
using nobleo_odometry_calibration::get_yaw;
using nobleo_odometry_calibration::inverse_odometry;
using nobleo_odometry_calibration::odometry;
using nobleo_odometry_calibration::to_2d;

constexpr auto eps = 1e-15;

struct DoubleParameterized : public ::testing::TestWithParam<double>
{
};

TEST(Math, create_quaternion_yaw)
{
  tf2::Quaternion q_tf2;
  q_tf2.setRPY(0, 0, 0.1);
  auto q_msg = tf2::toMsg(q_tf2);
  auto q1 = from_msg(q_msg);

  auto q2 = create_quaternion(0, 0, 0.1);

  EXPECT_NEAR(q1.x(), q2.x(), eps);
  EXPECT_NEAR(q1.y(), q2.y(), eps);
  EXPECT_NEAR(q1.z(), q2.z(), eps);
  EXPECT_NEAR(q1.w(), q2.w(), eps);
}

TEST(Math, create_quaternion_rpy)
{
  tf2::Quaternion q_tf2;
  q_tf2.setRPY(0.1, 0.2, 0.3);
  auto q_msg = tf2::toMsg(q_tf2);
  auto q1 = from_msg(q_msg);

  auto q2 = create_quaternion(0.1, 0.2, 0.3);

  EXPECT_NEAR(q1.x(), q2.x(), eps);
  EXPECT_NEAR(q1.y(), q2.y(), eps);
  EXPECT_NEAR(q1.z(), q2.z(), eps);
  EXPECT_NEAR(q1.w(), q2.w(), eps);
}

TEST_P(DoubleParameterized, get_yaw)
{
  double yaw = GetParam();
  tf2::Quaternion q1;
  q1.setRPY(0, 0, yaw);
  auto msg = tf2::toMsg(q1);
  auto q2 = from_msg(msg);

  EXPECT_NEAR(tf2::getYaw(q1), get_yaw(q2.toRotationMatrix()), eps);
}

TEST(Math, get_yaw_from_rpy)
{
  tf2::Quaternion q1;
  q1.setRPY(0.1, 0.2, 0.3);
  auto msg = tf2::toMsg(q1);
  auto q2 = from_msg(msg);

  EXPECT_NEAR(tf2::getYaw(q1), get_yaw(q2.toRotationMatrix()), eps);
}

TEST(Math, to_2d)
{
  double yaw = 0.3;
  auto transform_3d = create_transform(2, 3, 4, 0.1, 0.2, yaw);
  auto transform_2d = to_2d(transform_3d);

  // translation
  EXPECT_NEAR(transform_3d.translation().x(), transform_2d.translation().x(), eps);
  EXPECT_NEAR(transform_3d.translation().y(), transform_2d.translation().y(), eps);

  // with a 2d rotation, the upper corner should stay the same
  EXPECT_NEAR(transform_3d(0, 0), transform_2d(0, 0), eps);
  EXPECT_NEAR(transform_3d(0, 1), transform_2d(0, 1), eps);
  EXPECT_NEAR(transform_3d(1, 0), transform_2d(1, 0), eps);
  EXPECT_NEAR(transform_3d(1, 1), transform_2d(1, 1), eps);

  // also verify the affine part, there was a bug with initialization
  EXPECT_NEAR(transform_2d(2, 0), 0, eps);
  EXPECT_NEAR(transform_2d(2, 1), 0, eps);
  EXPECT_NEAR(transform_2d(2, 2), 1, eps);
}

TEST_P(DoubleParameterized, to_2d)
{
  double yaw = GetParam();
  auto transform_3d = create_transform(2, 0, 0, 0, 0, yaw);
  auto transform_2d = to_2d(transform_3d);

  // translation
  EXPECT_NEAR(transform_3d.translation().x(), transform_2d.translation().x(), eps);
  EXPECT_NEAR(transform_3d.translation().y(), transform_2d.translation().y(), eps);

  // with a 2d rotation, the upper corner should stay the same
  EXPECT_NEAR(transform_3d(0, 0), transform_2d(0, 0), eps);
  EXPECT_NEAR(transform_3d(0, 1), transform_2d(0, 1), eps);
  EXPECT_NEAR(transform_3d(1, 0), transform_2d(1, 0), eps);
  EXPECT_NEAR(transform_3d(1, 1), transform_2d(1, 1), eps);

  // also verify the affine part, there was a bug with initialization
  EXPECT_NEAR(transform_2d(2, 0), 0, eps);
  EXPECT_NEAR(transform_2d(2, 1), 0, eps);
  EXPECT_NEAR(transform_2d(2, 2), 1, eps);
}

TEST(Math, odometry)
{
  auto odom = odometry(2.0, 0.2);
  auto [linear, angular] = inverse_odometry(odom);

  EXPECT_NEAR(linear, 2, eps);
  EXPECT_NEAR(angular, 0.2, eps);
}

INSTANTIATE_TEST_SUITE_P(Math, DoubleParameterized, testing::Values(0.1, -0.1, 3.13, -3.14));
