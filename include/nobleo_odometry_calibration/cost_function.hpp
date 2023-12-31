// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <ceres/autodiff_cost_function.h>

#include "./math.hpp"
namespace nobleo_odometry_calibration
{
/**
 * @brief Sanity check that the matrix is still affine
 *
 * If the assert fails there is an uninitialized transform somewhere.
 */
template <typename T>
void assert_is_affine([[maybe_unused]] const Eigen::Transform<T, 2, Eigen::Isometry> & matrix)
{
  assert(matrix(2, 0) == T(0));
  assert(matrix(2, 1) == T(0));
  assert(matrix(2, 2) == T(1));
}

struct CostFunctor
{
public:
  CostFunctor(const Transform & sensor_diff, const Transform & odom_diff)
  : sensor_diff_(to_2d(sensor_diff)), odom_diff_(to_2d(odom_diff))
  {
  }

  template <typename T>
  bool operator()(
    const T * const sensor_x, const T * const sensor_y, const T * const sensor_theta,
    T * residual) const
  {
    auto sensor_pose = create_transform_2d(*sensor_x, *sensor_y, *sensor_theta);

    auto odom_diff_in_sensor = sensor_pose.inverse() * odom_diff_.cast<T>() * sensor_pose;

    auto err = sensor_diff_.inverse().cast<T>() * odom_diff_in_sensor;
    assert_is_affine(err);

    Eigen::Rotation2D<T> rot;
    rot.fromRotationMatrix(err.linear());
    residual[0] = err.translation().x();
    residual[1] = err.translation().y();
    residual[2] = rot.angle();
    return true;
  }

  template <typename T>
  bool operator()(
    const T * const sensor_x, const T * const sensor_y, const T * const sensor_theta,
    const T * const wheel_separation_multiplier, const T * const wheel_radius_multiplier,
    T * residual) const
  {
    auto [linear, angular] = inverse_odometry(odom_diff_);

    auto linear2 = linear * *wheel_radius_multiplier;
    auto angular2 = angular * *wheel_radius_multiplier / *wheel_separation_multiplier;

    auto new_odom_diff = odometry(linear2, angular2);

    auto sensor_pose = create_transform_2d(*sensor_x, *sensor_y, *sensor_theta);

    auto odom_diff_in_sensor = sensor_pose.inverse() * new_odom_diff * sensor_pose;

    auto err = sensor_diff_.inverse().cast<T>() * odom_diff_in_sensor;
    assert_is_affine(err);

    Eigen::Rotation2D<T> rot;
    rot.fromRotationMatrix(err.linear());
    residual[0] = err.translation().x();
    residual[1] = err.translation().y();
    residual[2] = rot.angle();
    return true;
  }

  static ceres::CostFunction * create(const Transform & sensor_diff, const Transform & odom_diff)
  {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 1, 1, 1>(
      new CostFunctor(sensor_diff, odom_diff));
  }

  static ceres::CostFunction * create5dof(
    const Transform & sensor_diff, const Transform & odom_diff)
  {
    return new ceres::AutoDiffCostFunction<CostFunctor, 3, 1, 1, 1, 1, 1>(
      new CostFunctor(sensor_diff, odom_diff));
  }

private:
  Eigen::Isometry2d sensor_diff_;
  Eigen::Isometry2d odom_diff_;
};

}  // namespace nobleo_odometry_calibration
