// Copyright 2013 Open Source Robotics Foundation
//
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <rosbag/bag.h>

#include <functional>

namespace nobleo_odometry_calibration
{
template <class T>
using BagCallbackT = std::function<void(const boost::shared_ptr<const T> &)>;

using BagCallback = std::function<void(const rosbag::MessageInstance &)>;

/**
 * @brief A class for playing back bag files
 *
 * It supports relatime, as well as accelerated and slowed playback.
 */
class BagPlayer
{
public:
  /**
   * @brief Constructor expecting the filename of a bag
   */
  explicit BagPlayer(const std::string & filename);

  /**
   * @brief Register a callback for a specific topic and type
   */
  template <class T>
  void register_callback(const std::string & topic, BagCallbackT<T> cb);
  void register_callback(const std::string & topic, BagCallback cb);

  /**
   * @brief Set the time in the bag to start
   *
   * Default is the first message
   */
  void set_start(const ros::Time & start);

  /**
   * @brief Set the time in the bag to stop
   *
   * Default is the last message
   */
  void set_end(const ros::Time & end);

  /**
   * @brief Set the speed to playback.
   *
   * 1.0 is the default. 2.0 would be twice as fast, 0.5 is half realtime.
   */
  void set_playback_speed(double scale);

  /**
   * @brief Start playback of the bag file using the parameters previously set
   */
  void start_play();

  const rosbag::Bag & bag() const { return bag_; }

private:
  ros::Time real_time(const ros::Time & msg_time) const;

  // The bag file interface loaded in the constructor.
  rosbag::Bag bag_;
  std::map<std::string, BagCallback> cbs_;
  ros::Time bag_start_;
  ros::Time bag_end_;
  ros::Time last_message_time_;
  double playback_speed_ = 1;
  ros::Time play_start_;
};

template <class T>
void BagPlayer::register_callback(const std::string & topic, BagCallbackT<T> cb)
{
  register_callback(topic, [cb](const rosbag::MessageInstance & m) {
    const auto msg = m.instantiate<T>();
    assert(msg);
    cb(msg);
  });
}
}  // namespace nobleo_odometry_calibration
