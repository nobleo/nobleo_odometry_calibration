// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/covariance.h>
#include <ceres/solver.h>
#include <ros/console.h>

#include <nobleo_odometry_calibration/cost_function.hpp>
#include <nobleo_odometry_calibration/solver.hpp>
#include <vector>

constexpr auto name = "solver";

namespace nobleo_odometry_calibration
{
// Normalizes the angle in radians between [-pi and pi).
template <typename T>
T NormalizeAngle(const T & angle_radians)
{
  T two_pi(2.0 * M_PI);
  return angle_radians - two_pi * ceres::floor((angle_radians + T(M_PI)) / two_pi);
}

class AngleLocalParameterization
{
public:
  template <typename T>
  bool operator()(
    const T * theta_radians, const T * delta_theta_radians, T * theta_radians_plus_delta) const
  {
    *theta_radians_plus_delta = NormalizeAngle(*theta_radians + *delta_theta_radians);
    return true;
  }

  static ceres::LocalParameterization * create()
  {
    return new ceres::AutoDiffLocalParameterization<AngleLocalParameterization, 1, 1>;
  }
};

Solver::Solver()
{
  problem_.AddParameterBlock(&parameters_.x, 1);
  problem_.AddParameterBlock(&parameters_.y, 1);
  problem_.AddParameterBlock(&parameters_.theta, 1, AngleLocalParameterization::create());
  problem_.AddParameterBlock(&parameters_.wheel_separation_multiplier, 1);
  problem_.AddParameterBlock(&parameters_.wheel_radius_multiplier, 1);
}

void Solver::configure(const OptimizeParameters & optimize)
{
  ROS_INFO_NAMED(
    name,
    "optimize parameters:\nx: %d\ny: %d\ntheta: %d\nwheel_separation_multiplier: "
    "%d\nwheel_radius_multiplier: %d",
    optimize.x, optimize.y, optimize.theta, optimize.wheel_separation_multiplier,
    optimize.wheel_radius_multiplier);

  // shorthand
  auto set_parameter_block = [this](double * values, bool variable) {
    if (variable)
      problem_.SetParameterBlockVariable(values);
    else
      problem_.SetParameterBlockConstant(values);
  };

  set_parameter_block(&parameters_.x, optimize.x);
  set_parameter_block(&parameters_.y, optimize.y);
  set_parameter_block(&parameters_.theta, optimize.theta);
  set_parameter_block(
    &parameters_.wheel_separation_multiplier, optimize.wheel_separation_multiplier);
  set_parameter_block(&parameters_.wheel_radius_multiplier, optimize.wheel_radius_multiplier);

  std::vector<ceres::ResidualBlockId> residual_blocks;
  problem_.GetResidualBlocks(&residual_blocks);
  if (!residual_blocks.empty()) ROS_WARN_NAMED(name, "Exising constraints will be removed");
  for (auto id : residual_blocks) {
    problem_.RemoveResidualBlock(id);
  }

  optimize_parameters_ = optimize;
}

void Solver::add_constraint(const Transform & sensor_diff, const Transform & odom_diff)
{
  if (
    optimize_parameters_.wheel_separation_multiplier ||
    optimize_parameters_.wheel_radius_multiplier) {
    problem_.AddResidualBlock(
      CostFunctor::create5dof(sensor_diff, odom_diff), nullptr, &parameters_.x, &parameters_.y,
      &parameters_.theta, &parameters_.wheel_separation_multiplier,
      &parameters_.wheel_radius_multiplier);
  } else {
    problem_.AddResidualBlock(
      CostFunctor::create(sensor_diff, odom_diff), nullptr, &parameters_.x, &parameters_.y,
      &parameters_.theta);
  }
}

[[nodiscard]] bool Solver::solve()
{
  if (problem_.NumResidualBlocks() <= 0) {
    ROS_ERROR_NAMED(name, "No constraints added");
    return false;
  }

  parameters_ = {};
  ROS_INFO_NAMED(
    name, "Initial parameters: x=%f y=%f theta=%f separation=%f radius=%f", parameters_.x,
    parameters_.y, parameters_.theta, parameters_.wheel_separation_multiplier,
    parameters_.wheel_radius_multiplier);

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);

  if (!summary.IsSolutionUsable()) {
    return false;
  }

  ROS_INFO_STREAM_NAMED(name, summary.BriefReport());
  ROS_DEBUG_STREAM_NAMED(name, summary.FullReport());
  ROS_INFO_NAMED(
    name, "Final parameters: x=%f y=%f theta=%f separation=%f radius=%f", parameters_.x,
    parameters_.y, parameters_.theta, parameters_.wheel_separation_multiplier,
    parameters_.wheel_radius_multiplier);

  ceres::Covariance covariance{{}};
  auto success = covariance.Compute(
    {&parameters_.x, &parameters_.y, &parameters_.theta, &parameters_.wheel_separation_multiplier,
     &parameters_.wheel_radius_multiplier},
    &problem_);
  if (!success) {
    ROS_WARN_NAMED(name, "Covariance computation failed");
    return false;
  }

  Eigen::Matrix<double, 5, 5> matrix;
  covariance.GetCovarianceMatrix(
    {&parameters_.x, &parameters_.y, &parameters_.theta, &parameters_.wheel_separation_multiplier,
     &parameters_.wheel_radius_multiplier},
    matrix.data());
  ROS_INFO_STREAM("Parameter covariance:\n" << matrix);

  return true;
}

std::vector<std::array<double, 3>> Solver::residuals()
{
  std::vector<double> residuals;
  problem_.Evaluate({}, nullptr, &residuals, nullptr, nullptr);

  std::vector<std::array<double, 3>> result;
  for (size_t i = 0; i < residuals.size() / 3; ++i) {
    result.push_back({residuals[3 * i], residuals[3 * i + 1], residuals[3 * i + 2]});
  }
  return result;
}
}  // namespace nobleo_odometry_calibration
