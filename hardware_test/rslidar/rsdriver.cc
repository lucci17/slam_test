/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009-2012 Austin Robot Technology, Jack O'Quin
 *	Copyright (C) 2017 Robosense, Tony Zhang
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver implementation for the RILIDAR 3D LIDARs
 */
#include "rsdriver.h"

namespace hardware
{
namespace rslidar
{
namespace driver
{

rslidarDriver::rslidarDriver(const Config &config)
    : config_(config)
{
  // TODO tf
  // 	std::string tf_prefix = tf::getPrefixParam(private_nh);
  // 	ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);
  // 	config_.frame_id = tf::resolve(tf_prefix, config_.frame_id);

  // get model name, validate string, determine packet rate
  double packet_rate; // packet frequency (Hz)
  std::string model_full_name;

  // product model
  if (config_.model == "RS16") {
    packet_rate = 840;
    model_full_name = "RS-LiDAR-16";
  } else if (config_.model == "RS32") {
    packet_rate = 1690;
    model_full_name = "RS-LiDAR-32";
  } else {
    PRINT_WARNING_FMT("unknown LIDAR model: %s", config_.model.c_str());
    packet_rate = 2600.0;
  }

  std::string deviceName(std::string("Robosense ") + model_full_name);
  double frequency = config_.rpm / 60.0; // expected Hz rate
  config_.npackets = (int32_t)ceil(packet_rate / frequency);
  PRINT_INFO_FMT("publishing %d packets per scan.", config_.npackets);

  // read data from live socket
  input_.reset(new InputSocket(config_.ip, config.port));
}

/** poll the device
 *
 *  @returns true unless end of file reached
 */
bool rslidarDriver::poll(RslidarScanPtr &scan)
{
  scan->packets.resize(config_.npackets);
  // Since the rslidar delivers data at a very high rate, keep
  // reading and publishing scans as fast as possible.
  for (int i = 0; i < config_.npackets; i++) {
    while (true) {
      // keep reading until full packet received
      int rc = input_->getPacket(&scan->packets[i], config_.time_offset);
      if (rc == 0)
        break; // got a full packet?
      if (rc > 0)
        return false; // end of file reached?
    }
  }

  // publish message using time of last packet read
  scan->header.stamp = scan->packets[config_.npackets - 1].stamp;
  scan->header.frame_id = config_.frame_id;
  return true;
}

void rslidarDriver::setTimeOffset(double offset)
{
  config_.time_offset = offset;

  if (config_.time_offset < -1.)
    config_.time_offset = -1.;
  else if (config_.time_offset > 1.)
    config_.time_offset = 1.;
}

} // namespace rs_driver
} // namespace rslidar
} // namespace hardware
