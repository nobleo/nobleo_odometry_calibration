#include "./solver.hpp"

#include <ceres/autodiff_local_parameterization.h>
#include <ceres/solver.h>
#include <ros/console.h>

#include <vector>

#include "./cost_function.hpp"

constexpr auto name = "solver";

namespace nobleo_gps_calibration
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

  problem_.SetParameterBlockConstant(&parameters_.x);
  problem_.SetParameterBlockConstant(&parameters_.y);
}

void Solver::configure(const Solver::Config & config)
{
  switch (config.optimize_parameters) {
    case OptimizeParameters::THETA:
      problem_.SetParameterBlockConstant(&parameters_.x);
      problem_.SetParameterBlockConstant(&parameters_.y);
      break;
    case OptimizeParameters::XTHETA:
      problem_.SetParameterBlockVariable(&parameters_.x);
      problem_.SetParameterBlockConstant(&parameters_.y);
      break;
    case OptimizeParameters::XYTHETA:
      problem_.SetParameterBlockVariable(&parameters_.x);
      problem_.SetParameterBlockVariable(&parameters_.y);
      break;
  }

  config_ = config;
}

void Solver::add_constraint(const Transform & gps_diff, const Transform & odom_diff)
{
  problem_.AddResidualBlock(
    CostFunctor::create(gps_diff, odom_diff), nullptr, &parameters_.x, &parameters_.y,
    &parameters_.theta);
}

void Solver::solve()
{
  parameters_ = {};

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem_, &summary);

  if (!summary.IsSolutionUsable()) {
    throw std::runtime_error("Ceres could not find a usable solution to optimize.");
  }

  ROS_INFO_STREAM_NAMED(name, summary.BriefReport());
  ROS_DEBUG_STREAM_NAMED(name, summary.FullReport());
  ROS_INFO_NAMED(
    name, "Final parameters: x=%f y=%f theta=%f", parameters_.x, parameters_.y, parameters_.theta);
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
}  // namespace nobleo_gps_calibration
