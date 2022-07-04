#pragma once
#include <ceres/problem.h>

#include "./config.hpp"
#include "./math.hpp"
namespace nobleo_gps_calibration
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

  void configure(const OptimizeParameters & optimize);
  void add_constraint(const Transform & gps_diff, const Transform & odom_diff);
  [[nodiscard]] bool solve();
  Parameters parameters() const { return parameters_; }
  std::vector<std::array<double, 3>> residuals();

private:
  OptimizeParameters optimize_parameters_;
  ceres::Problem problem_;
  Parameters parameters_;
};
}  // namespace nobleo_gps_calibration
