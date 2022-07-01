/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Open Source Robotics Foundation
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#pragma once

#include <rosbag/bag.h>

#include <functional>

namespace nobleo_gps_calibration
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
  cbs_[topic] = [cb](const rosbag::MessageInstance & m) {
    const auto msg = m.instantiate<T>();
    assert(msg);
    cb(msg);
  };
}
}  // namespace nobleo_gps_calibration
