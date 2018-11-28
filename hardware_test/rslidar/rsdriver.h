/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file rsdriver.h
 *
 *  ROS driver interface for the RSLIDAR 3D LIDARs
 */
#pragma once

#include "input.h"
#include <pcl/point_types.h>
#include <string>

namespace hardware
{
/*!
 * @namespace hardware::rslidar
 * @brief namespace for all class and namespace defined for Robosense device
 */
namespace rslidar
{
namespace driver
{

struct Config {
  std::string frame_id; ///< tf frame ID
  std::string model;    ///< device model name
  std::string ip;
  int32_t npackets; ///< number of packets to collect
  int32_t port;
  double rpm;         ///< device rotation rate (RPMs)
  double time_offset; ///< time in seconds added to each  time stamp
};

static const Config default_config = {.frame_id = "horizontal_laser_link",
                                      .model = "RS16",
                                      .ip = "192.168.1.200",
                                      .npackets = 0,
                                      .port = (int32_t)DATA_PORT_NUMBER,
                                      .rpm = 600.,
                                      .time_offset = 0. };

class rslidarDriver
{
public:
  /**
 * @brief rslidarDriver
 */
  rslidarDriver()
      : config_(default_config)
  {
  }

  rslidarDriver(const Config &config);

  ~rslidarDriver() {}

  void setTimeOffset(double offset);

  inline void setFrameId(std::string frame_id) { config_.frame_id = frame_id; }

  bool poll(RslidarScanPtr &scan);

private:
  // configuration parameters
  Config config_;
  boost::shared_ptr<Input> input_;

  double diag_min_freq_;
  double diag_max_freq_;
};

} // namespace rs_driver
} // namespace rslidar
} // namespace hardware
