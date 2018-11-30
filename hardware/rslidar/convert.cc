/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw RSLIDAR 3D LIDAR packets to PointCloud2.

*/
#include "convert.h"
namespace hardware
{
namespace rslidar
{
namespace point_cloud
{

/** @brief Constructor. */
Convert::Convert(const rawdata::Config &config)
    : data_(new rawdata::RawData())
{
  data_->loadConfigFile(config); // load lidar parameters
  data_->init_setup();
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(
  const hardware::rslidar::msgs::RslidarScanConstPtr &scanMsg,
  Bgs::PointCloud2Ptr &msg)
{
  msg->header = Bgs::HeaderFromBgsToPcl(scanMsg->header);

  // process each packet provided by the driver
  bool finish_packets_parse = false;
  for (size_t i = 0; i < scanMsg->packets.size(); ++i) {
    if (i == (scanMsg->packets.size() - 1)) {
      // ROS_INFO_STREAM("Packets per scan: "<< scanMsg->packets.size());
      finish_packets_parse = true;
    }

    data_->unpack(scanMsg->packets[i], msg, finish_packets_parse);
  }
}
} // namespace point_cloud
}
}
