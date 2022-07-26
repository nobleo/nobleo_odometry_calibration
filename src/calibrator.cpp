#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/node_handle.h>
#include <tf2/buffer_core.h>
#include <tf2/exceptions.h>

#include <memory>
#include <nobleo_gps_calibration/calibrator.hpp>

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

namespace nobleo_gps_calibration
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
  if (
    gps_diff.translation().norm() < config_.update_min_d &&
    std::abs(get_yaw(gps_diff.linear())) < config_.update_min_a) {
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
  marker_.header.stamp = gps->header.stamp;
  marker_.colors.push_back(create_rgba(0, 1, 0));
  marker_.points.push_back(to_msg(last_gps_pose_->translation()));
  marker_.colors.push_back(create_rgba(0, 1, 1));
  marker_.points.push_back(to_msg(gps_pose.translation()));
  marker_.colors.push_back(create_rgba(1, 0, 0));
  marker_.points.push_back(to_msg(gps_pose.translation()));
  marker_.colors.push_back(create_rgba(1, 0, 0));
  marker_.points.push_back(to_msg(*last_gps_pose_ * odom_diff.translation()));
  marker_pub_.publish(marker_);

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
