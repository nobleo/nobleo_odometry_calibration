// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <ceres/problem.h>

#include "./config.hpp"
#include "./math.hpp"
namespace nobleo_odometry_calibration
{
class Solver
{
public:
  struct Parameters
  {
    double x = 0;
    double y = 0;
    double theta = 0;
    double wheel_separation_multiplier = 1;
    double wheel_radius_multiplier = 1;

    Parameters() = default;
  };

  Solver();

  /**
   * @brief Reconfigure the solver
   * @param optimize Which parameters to optimize
   */
  void configure(const OptimizeParameters & optimize);

  /**
   * @brief Add a measurement to include in the optimization process
   * @param sensor_diff A position delta measured by the sensor
   * @param odom_diff A position delta measured bye the odometry sensor
   */
  void add_constraint(const Transform & sensor_diff, const Transform & odom_diff);

  /**
   * @brief Solve the optimization problem
   *
   * After a `solve` call, the parameters will be optimized such that the residuals are as small as
   * possible.
   */
  [[nodiscard]] bool solve();

  /**
   * @brief Get the current value of the parameters that are optimized
   */
  Parameters parameters() const { return parameters_; }

  /**
   * @brief Calculate the residuals with the current parameter values
   *
   * The residuals are the errors that are calculated by the loss function. Each constraint adds one
   * or more residuals. The components of the residual are dx, dy and dtheta. After a `solve` call,
   * they should be a lot smaller.
   */
  std::vector<std::array<double, 3>> residuals();

private:
  OptimizeParameters optimize_parameters_;
  ceres::Problem problem_;
  Parameters parameters_;
};
}  // namespace nobleo_odometry_calibration
