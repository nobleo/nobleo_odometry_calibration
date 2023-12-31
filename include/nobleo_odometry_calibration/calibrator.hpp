// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include <ros/publisher.h>
#include <tf2/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include <optional>

#include "./config.hpp"
#include "./solver.hpp"

namespace nav_msgs
{
ROS_DECLARE_MESSAGE(Odometry)
}

namespace tf2
{
class BufferCore;
}
namespace nobleo_odometry_calibration
{
class Calibrator
{
public:
  Calibrator(ros::NodeHandle & nh, const std::shared_ptr<tf2::BufferCore> & buffer);

  void configure(const CalibratorConfig & config);
  void add(const nav_msgs::OdometryConstPtr & sensor);
  [[nodiscard]] bool solve() { return solver_.solve(); }
  Solver::Parameters parameters() const { return solver_.parameters(); }
  std::vector<std::array<double, 3>> residuals() { return solver_.residuals(); };

private:
  Transform get_odom_pose(const ros::Time & time) const;

  const std::shared_ptr<const tf2::BufferCore> buffer_;
  std::optional<tf2::Stamped<Transform>> last_sensor_pose_;
  Solver solver_;
  Config config_;

  visualization_msgs::Marker marker_;
  ros::Publisher marker_pub_;
};
}  // namespace nobleo_odometry_calibration
