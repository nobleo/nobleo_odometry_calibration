// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

#include <memory>
#include <nobleo_odometry_calibration/calibrator.hpp>

namespace
{
constexpr auto name = "calibrator";

std_msgs::ColorRGBA create_rgba(float r, float g, float b, float a = 1)
{
  std_msgs::ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = a;
  return result;
}
}  // namespace

namespace nobleo_odometry_calibration
{
Calibrator::Calibrator(ros::NodeHandle & nh, const std::shared_ptr<tf2::BufferCore> & buffer)
: buffer_(buffer), marker_pub_(nh.advertise<visualization_msgs::Marker>("visualization", 1))
{
  marker_.type = visualization_msgs::Marker::LINE_LIST;
  marker_.action = visualization_msgs::Marker::MODIFY;
  marker_.pose.orientation.w = 1;
  marker_.color.g = 1;
  marker_.color.a = 1;
  marker_.scale.x = 0.05;
}

void Calibrator::configure(const CalibratorConfig & config)
{
  config_ = Config{config};
  solver_.configure(config_.optimize_parameters);

  marker_.header.frame_id = config_.global_frame_id;
}

void Calibrator::add(const nav_msgs::OdometryConstPtr & sensor)
{
  Transform odom_pose;
  auto sensor_pose = from_msg(sensor->pose.pose);

  try {
    odom_pose = get_odom_pose(sensor->header.stamp);
  } catch (const tf2::ExtrapolationException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  if (!last_sensor_pose_) {
    ROS_INFO_NAMED(name, "first sensor update, recording starting pose");
    last_sensor_pose_ =
      tf2::Stamped<Transform>{sensor_pose, sensor->header.stamp, sensor->header.frame_id};
    return;
  }

  auto sensor_diff = last_sensor_pose_->inverse() * sensor_pose;
  if (
    sensor_diff.translation().norm() < config_.update_min_d &&
    std::abs(get_yaw(sensor_diff.linear())) < config_.update_min_a) {
    return;
  }

  ROS_DEBUG_NAMED(
    name, "sensor_diff: x=%f y=%f t=%f", sensor_diff.translation().x(),
    sensor_diff.translation().y(), get_yaw(sensor_diff.linear()));

  Transform last_odom_pose;
  try {
    last_odom_pose = get_odom_pose(last_sensor_pose_->stamp_);
  } catch (const tf2::ExtrapolationException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  auto odom_diff = last_odom_pose.inverse() * odom_pose;
  ROS_DEBUG_NAMED(
    name, "odom_diff: x=%f y=%f t=%f", odom_diff.translation().x(), odom_diff.translation().y(),
    get_yaw(odom_diff.linear()));
  marker_.header.stamp = sensor->header.stamp;
  marker_.colors.push_back(create_rgba(0, 1, 0));
  marker_.points.push_back(to_msg(last_sensor_pose_->translation()));
  marker_.colors.push_back(create_rgba(0, 1, 1));
  marker_.points.push_back(to_msg(sensor_pose.translation()));
  marker_.colors.push_back(create_rgba(1, 0, 0));
  marker_.points.push_back(to_msg(sensor_pose.translation()));
  marker_.colors.push_back(create_rgba(1, 0, 0));
  marker_.points.push_back(to_msg(*last_sensor_pose_ * odom_diff.translation()));
  marker_pub_.publish(marker_);

  solver_.add_constraint(sensor_diff, odom_diff);

  last_sensor_pose_ =
    tf2::Stamped<Transform>{sensor_pose, sensor->header.stamp, sensor->header.frame_id};
}

Transform Calibrator::get_odom_pose(const ros::Time & time) const
{
  // don't use .transform() because this could run offline without a listener thread
  auto tf = buffer_->lookupTransform(config_.odom_frame_id, config_.base_frame_id, time);
  return from_msg(tf.transform);
}
}  // namespace nobleo_odometry_calibration
