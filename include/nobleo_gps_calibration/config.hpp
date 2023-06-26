// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

namespace nobleo_gps_calibration
{
class CalibratorConfig;

struct OptimizeParameters
{
  bool x = false;
  bool y = false;
  bool theta = false;
  bool wheel_separation_multiplier = false;
  bool wheel_radius_multiplier = false;

  OptimizeParameters() = default;
};

struct Config
{
  Config() = default;
  explicit Config(const CalibratorConfig & config);

  double update_min_d;
  double update_min_a;
  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;
  OptimizeParameters optimize_parameters;
};
}  // namespace nobleo_gps_calibration
