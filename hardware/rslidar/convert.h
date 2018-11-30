/* -*- mode: C++ -*- */
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

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include "conversions.h"
#include "rawdata.h"
#include "rslidar_msgs.h"

namespace hardware
{
namespace rslidar
{
namespace point_cloud
{

class Convert
{
public:
  Convert(const rawdata::Config &config);
  ~Convert() {}

  void processScan(const msgs::RslidarScanConstPtr &scanMsg,
                   Bgs::PointCloud2Ptr &msg);

private:
  boost::shared_ptr<rawdata::RawData> data_;
};

} // namespace point_cloud
}
}
#endif
