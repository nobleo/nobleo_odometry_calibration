#include "./calibrator.hpp"

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

#include <memory>

constexpr auto name = "calibrator";
namespace nobleo_gps_calibration
{
Calibrator::Calibrator(const std::shared_ptr<tf2::BufferCore> & buffer) : buffer_(buffer) {}

void Calibrator::configure(const CalibratorConfig & config)
{
  config_ = Config{config};
  solver_.configure(config_.optimize_parameters);
}

void Calibrator::add(const nav_msgs::OdometryConstPtr & gps)
{
  Transform odom_pose;
  auto gps_pose = from_msg(gps->pose.pose);

  try {
    odom_pose = get_odom_pose(gps->header.stamp);
  } catch (const tf2::ExtrapolationException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  if (!last_gps_pose_) {
    ROS_INFO_NAMED(name, "first gps update, recording starting pose");
    last_gps_pose_ = tf2::Stamped<Transform>{gps_pose, gps->header.stamp, gps->header.frame_id};
    return;
  }

  auto gps_diff = last_gps_pose_->inverse() * gps_pose;
  if (gps_diff.translation().norm() < config_.update_min_d) {
    return;
  }

  ROS_DEBUG_NAMED(
    name, "gps_diff: x=%f y=%f t=%f", gps_diff.translation().x(), gps_diff.translation().y(),
    get_yaw(gps_diff.linear()));

  Transform last_odom_pose;
  try {
    last_odom_pose = get_odom_pose(last_gps_pose_->stamp_);
  } catch (const tf2::ExtrapolationException & e) {
    ROS_WARN_NAMED(name, "failed to compute odom pose, skipping measurement (%s)", e.what());
    return;
  }

  auto odom_diff = last_odom_pose.inverse() * odom_pose;
  ROS_DEBUG_NAMED(
    name, "odom_diff: x=%f y=%f t=%f", odom_diff.translation().x(), odom_diff.translation().y(),
    get_yaw(odom_diff.linear()));

  solver_.add_constraint(gps_diff, odom_diff);

  last_gps_pose_ = tf2::Stamped<Transform>{gps_pose, gps->header.stamp, gps->header.frame_id};
}

Transform Calibrator::get_odom_pose(const ros::Time & time) const
{
  // don't use .transform() because this could run offline without a listener thread
  auto tf = buffer_->lookupTransform(config_.odom_frame_id, config_.base_frame_id, time);
  return from_msg(tf.transform);
}
}  // namespace nobleo_gps_calibration
