#pragma once
#include <nobleo_gps_calibration/CalibratorConfig.h>
#include <tf2/transform_datatypes.h>

#include <optional>

#include "./solver.hpp"

namespace nav_msgs
{
ROS_DECLARE_MESSAGE(Odometry)
}

namespace tf2
{
class BufferCore;
}
namespace nobleo_gps_calibration
{
struct Config
{
  Config() = default;
  explicit Config(const CalibratorConfig & config);

  double update_min_d;
  double update_min_a;
  std::string odom_frame_id;
  std::string base_frame_id;
  std::string global_frame_id;
};

class Calibrator
{
public:
  explicit Calibrator(const std::shared_ptr<tf2::BufferCore> & buffer);

  void configure(const CalibratorConfig & config);
  void add(const nav_msgs::OdometryConstPtr & gps);
  [[nodiscard]] bool solve() { return solver_.solve(); }
  Solver::Parameters parameters() const { return solver_.parameters(); }
  std::vector<std::array<double, 3>> residuals() { return solver_.residuals(); };

private:
  Transform get_odom_pose(const ros::Time & time) const;

  const std::shared_ptr<const tf2::BufferCore> buffer_;
  std::optional<tf2::Stamped<Transform>> last_gps_pose_;
  Solver solver_;
  Config config_;
};
}  // namespace nobleo_gps_calibration
