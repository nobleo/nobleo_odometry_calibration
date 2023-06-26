// Copyright 2022 Nobleo Technology B.V.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <tf2/buffer_core.h>

namespace rosbag
{
class Bag;
}

namespace nobleo_gps_calibration
{
/**
 * @brief Load all tf messages from a rosbag into a tf2::BufferCore
 */
class BagBuffer : public tf2::BufferCore
{
public:
  explicit BagBuffer(const rosbag::Bag & bag);
};

}  // namespace nobleo_gps_calibration
