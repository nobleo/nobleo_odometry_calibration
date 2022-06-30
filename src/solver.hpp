#pragma once
#include <ceres/problem.h>

#include "./math.hpp"

namespace nobleo_gps_calibration
{
class Solver
{
public:
  enum class OptimizeParameters {
    THETA,
    XTHETA,
    XYTHETA,
  };

  struct Config
  {
    OptimizeParameters optimize_parameters = OptimizeParameters::THETA;
  };

  struct Parameters
  {
    double x;
    double y;
    double theta;
  };

  Solver();

  void configure(const Config & config);
  void add_constraint(const Transform & gps_diff, const Transform & odom_diff);
  [[nodiscard]] bool solve();
  Parameters parameters() const { return parameters_; }
  std::vector<std::array<double, 3>> residuals();

private:
  Config config_;
  ceres::Problem problem_;
  Parameters parameters_ = {};
};
}  // namespace nobleo_gps_calibration
