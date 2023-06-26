// Copyright 2013 Open Source Robotics Foundation
//
// SPDX-License-Identifier: BSD-3-Clause

#include <ros/init.h>
#include <ros/names.h>
#include <rosbag/view.h>

#include <nobleo_gps_calibration/bag_player.hpp>
#include <string>
#include <vector>

constexpr auto name = "bag_player";

namespace nobleo_gps_calibration
{
BagPlayer::BagPlayer(const std::string & filename)
{
  bag_.open(filename, rosbag::bagmode::Read);
  ros::Time::init();
  rosbag::View v(bag_);
  bag_start_ = v.getBeginTime();
  bag_end_ = v.getEndTime();
}

void BagPlayer::register_callback(const std::string & topic, BagCallback cb)
{
  auto resolved_name = ros::names::resolve(topic);
  ROS_DEBUG_NAMED(name, "resolving topic '%s' as '%s'", topic.c_str(), resolved_name.c_str());
  cbs_[resolved_name] = cb;
}

void BagPlayer::set_start(const ros::Time & start) { bag_start_ = start; }

void BagPlayer::set_end(const ros::Time & end) { bag_end_ = end; }

void BagPlayer::set_playback_speed(double scale)
{
  if (scale > 0.0) playback_speed_ = scale;
}

ros::Time BagPlayer::real_time(const ros::Time & msg_time) const
{
  return play_start_ + (msg_time - bag_start_) * (1 / playback_speed_);
}

void BagPlayer::start_play()
{
  std::vector<std::string> topics;
  for (const auto & [topic, cb] : cbs_) topics.push_back(topic);

  rosbag::View view(bag_, rosbag::TopicQuery(topics), bag_start_, bag_end_);
  play_start_ = ros::Time::now();

  for (rosbag::MessageInstance const & m : view) {
    if (!ros::ok()) break;

    if (cbs_.find(m.getTopic()) == cbs_.end()) continue;

    ros::Time::sleepUntil(real_time(m.getTime()));
    ros::spinOnce();  // handle dynamic reconfigure calls

    last_message_time_ = m.getTime(); /* this is the recorded time */
    auto cb = cbs_[m.getTopic()];
    cb(m);
  }
}
}  // namespace nobleo_gps_calibration
