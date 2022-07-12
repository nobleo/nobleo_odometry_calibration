#include <nobleo_gps_calibration/CalibratorConfig.h>

#include <nobleo_gps_calibration/config.hpp>

namespace nobleo_gps_calibration
{
Config::Config(const CalibratorConfig & config)
: update_min_d(config.update_min_d),
  update_min_a(config.update_min_a),
  odom_frame_id(config.odom_frame_id),
  base_frame_id(config.base_frame_id),
  global_frame_id(config.global_frame_id)
{
  optimize_parameters.x = config.x;
  optimize_parameters.y = config.y;
  optimize_parameters.theta = config.theta;
  optimize_parameters.wheel_separation_multiplier = config.wheel_separation_multiplier;
  optimize_parameters.wheel_radius_multiplier = config.wheel_radius_multiplier;
}
}  // namespace nobleo_gps_calibration
