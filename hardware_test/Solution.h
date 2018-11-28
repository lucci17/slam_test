/*!
 * @file Solution.h
 * @brief Head file for LidarCollectors.
 *
 * @author zong.zhang.lin
 * @date 2018-10-06
 */

#pragma once

#include "struct_defines.h"

namespace hardware
{

/*!
 * @namespace hardware::gnss
 * @brief namespace for gps class and operations
 */
namespace gnss
{

typedef Bgs::Point ecef_t;

/*!
 * @brief gps position data
 *
 * including position (3d) and gps mode ( fixed or not ),
 * the position is a relative coordinate ( RTK GPS )
 */
struct GPSData {
  ecef_t enu;
  int mode;
  void Zerolizing()
  {
    enu.x = 0.0;
    enu.y = 0.0;
    enu.z = 0.0;
    mode = 0;
  }
};

/*!
 * @brief gps gesture data
 *
 * if you have 2 antennas for your RTK device,
 * you can get yaw and tile from the device
 */
struct GPSGesture {
  float yaw;
  float tile;
  int mode;
  void Zerolizing()
  {
    yaw = 0.0;
    tile = 0.0;
    mode = 0;
  }
};

/*!
 * @class Solution
 * @brief class Solution
 *
 * @details
 * this class is a decoder for GPS data frames,
 * it can decode ( for now ):
 * - GPGGA & GNGGA
 * - GPNTR
 * - PTNL_PJK
 * - PTNL_VGK
 * - PTNL_AVR
 *
 * and turn them into position data and gesture data
 */
class Solution
{
public:
  /*!
   * @brief run a loop to read every byte in the buffer
   *
   * if got a "\n", the decode the whole frame using
   * the decodeLine function and frame decoding functions
   *
   * @param buffer pointer to the data got from serial
   * @param length length of the data
   *
   * @return
   * - BGS_ERROR : got a wrong message
   * - >= 0 : buffer size
   */
  int readByte(const char *buffer, int length);

  /// \brief seperate the whole frame into several strings
  std::vector<std::string> decodeLine(std::string line);

  GPSGesture getGestureData() { return gestureData; }
  GPSData getPositionData() { return positionData; }

private:
  std::string readBuffer;
  GPSData positionData;
  GPSGesture gestureData;
  uint32_t max_size_ = 1024;

  int decodeGPGGA(const std::vector<std::string> &parameters);

  int decodeGPNTR(const std::vector<std::string> &parameters);

  int decodePTNL_PJK(const std::vector<std::string> &parameters);

  int decodePTNL_VGK(const std::vector<std::string> &parameters);

  int decodePTNL_AVR(const std::vector<std::string> &parameters);

  int decodePTNL_VHD(const std::vector<std::string> &parameters);
};

} // namespace gnss
} // namespace hardware
