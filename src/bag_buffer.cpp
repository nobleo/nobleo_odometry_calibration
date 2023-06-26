// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>
#include <tf2_msgs/TFMessage.h>

#include <nobleo_gps_calibration/bag_buffer.hpp>
#include <string>
#include <vector>

constexpr auto name = "bag_buffer";
namespace nobleo_gps_calibration
{
ros::Duration duration_from_bag(const rosbag::Bag & bag)
{
  std::vector<ros::Time> ts;
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery("/tf"))) {
    tf2_msgs::TFMessageConstPtr tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr) throw std::invalid_argument("Received /tf message with the wrong type");
    for (const auto & tf : tfs->transforms) ts.push_back(tf.header.stamp);
  }

  ROS_INFO_NAMED(name, "Loaded %zu messages from /tf", ts.size());
  if (ts.empty()) throw std::invalid_argument("Can't find any /tf messages in the bagfile");

  const auto [min, max] = std::minmax_element(ts.begin(), ts.end());
  ROS_INFO_NAMED(name, "Loaded /tf messages, first=%f last=%f", min->toSec(), max->toSec());
  return *max - *min;
}

BagBuffer::BagBuffer(const rosbag::Bag & bag) : BufferCore(duration_from_bag(bag))
{
  std::vector<std::string> topics = {"/tf", "/tf_static"};
  for (const auto & msg : rosbag::View(bag, rosbag::TopicQuery(topics))) {
    auto tfs = msg.instantiate<tf2_msgs::TFMessage>();
    if (tfs == nullptr) throw std::invalid_argument("Received /tf message with the wrong type");
    for (const auto & tf : tfs->transforms) {
      this->setTransform(tf, "bagfile", msg.getTopic() == "/tf_static");
    }
  }
}
}  // namespace nobleo_gps_calibration
