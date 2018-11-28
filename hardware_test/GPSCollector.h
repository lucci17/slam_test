/*!
 * @file GPSCollector.h
 * @brief Head file for GPSCollector.
 *
 * @author edward.liu
 * @date 2018-10-08
 */

#pragma once

#include "Solution.h"
#include "bgs_time.h"
#include "struct_defines.h"

#include "macro_defines.h"
#include "serial.h"
#include <boost/function.hpp>

namespace hardware
{

/*!
 * @brief class GPSCollector
 *
 * usually, we use the gps sensor in asynchronized mode
 * so, the callback function should be set
 * more details can be found in class SerialPort
 *
 * @details
 * there are 2 callback function in GPSCollector
 * - one is for serial port( SerialPortCallback )
 * - the other is for gps collector( GpsCallback )
 *
 * some times, we get some messages from serialport, the SerialPortCallback
 * function called but we do not get complete GPGGA(for instance) frame, so
 * it is not time to call GpsCallback. we will wait for a complete frame to
 * call GpsCallback function.
 *
 * this is shown in the init() function.
 */
class GPSCollector
{
public:
  using GpsCallback = boost::function<int32_t(gnss::GPSData, gnss::GPSGesture)>;

  GPSCollector() {}
  ~GPSCollector() {}

  /// \brief Init serial port with the device path
  int32_t init(char *path_name);

  /// \brief Deinit serial port
  void deInit();

  void set_callback(const GpsCallback &callback);

private:
  /*!
   * @brief solution used for decoding arrived data
   *
   * the arrived message is in ASCII
   * and should be decoded into binary data,
   * more details in class gnss::Solution
   */
  gnss::Solution solution_;

  SerialPortPtr serial_port_;

  /// \brief callback funtion called when new gps data arrives
  GpsCallback callback_; // 获取到新的数据之后的回调函数

  /// \brief if the callback function has been set
  bool has_callback_ = false;
};

} // namespace hardware