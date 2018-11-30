#ifndef N301_PACKET_MSG_H
#define N301_PACKET_MSG_H

#include <boost/array.hpp>
#include <stddef.h>
#include <vector>

struct N301_packet {
  char data[1206];
};

struct N301_point {
  float time;
  double x;
  double y;
  double z;
  double azimuth;
  double distance;
  double intensity;
};

struct N301_scan {
  double altitude;
  std::vector<N301_point> points;
};

struct N301_sweep {
  boost::array<N301_scan, 16> scans;
};

struct N301_lidar {
  int32_t num_ranges;
  std::vector<double> azimuth;
  std::vector<double> ranges;
  std::vector<double> intensities;
  std::vector<double> x;
  std::vector<double> y;

  N301_lidar operator=(N301_lidar &lidar)
  {
    this->num_ranges = lidar.num_ranges;
    this->azimuth.assign(lidar.azimuth.begin(), lidar.azimuth.end());
    this->ranges.assign(lidar.ranges.begin(), lidar.ranges.end());
    this->intensities.assign(lidar.intensities.begin(),
                             lidar.intensities.end());
    this->x.assign(lidar.x.begin(), lidar.x.end());
    this->y.assign(lidar.y.begin(), lidar.y.end());

    return *this;
  }
};

#endif // N301_PACKET_MSG_H
