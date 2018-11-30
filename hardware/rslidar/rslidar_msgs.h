#pragma once

#include "bgs_time.h"
#include "macro_defines.h"
#include "struct_defines.h"

namespace hardware
{
namespace rslidar
{
namespace msgs
{

struct RslidarPic {
  Bgs::Header header;
  uint32_t col;
  std::vector<float> distance;
  std::vector<float> intensity;
  std::vector<float> azimuthforeachP;
};

struct RslidarPacket {
  BgsTime stamp;
  uint8_t data[1248];
};

struct RslidarScan {
  Bgs::Header header;
  std::vector<RslidarPacket> packets;
};
typedef std::shared_ptr<RslidarScan> RslidarScanPtr;
typedef std::shared_ptr<RslidarScan const> RslidarScanConstPtr;

} // namespace msg
} // namespace rslidar
} // namespace hardware
