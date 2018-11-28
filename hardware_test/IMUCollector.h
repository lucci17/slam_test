#pragma once

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"
#include "struct_defines.h"

enum ImuType { kLpms, kMPU9250, kImuTypeCount };

namespace hardware
{

/*!
 * @class IMUCollector
 * @brief interface for all imu collector
 *
 * it is a virtual class
 */
class IMUCollector
{
public:
  IMUCollector(){};
  ~IMUCollector(){};

  /*!
   * @brief initilisation with a device path
   *
   * @param path device path lick "/dev/ttyUSB0"
   */
  virtual void init(char path[]) = 0;

  /*!
   * @brief reset the device
   * should be called before destroying the instance
   */
  virtual void deInit() = 0;

  /*!
   * @brief get current imu data
   *
   * @param imu_msg return the data into this pointer
   *
   * @return
   * - BGS_OK : succeed
   * - BGS_ERROR : failed
   */
  virtual int32_t get_imu_msg(Bgs::ImuMsg *imu_msg) = 0;
};

/*!
 * @brief data transform ImuData->ImuMsg
 *
 * @param data the type "ImuData" is from lpsensor libs
 * @param imu_msg the type "ImuMsg" is for our own use, it
 * is like the type in ros
 */
void ImuData_to_ImuMsg(const ImuData &data, Bgs::ImuMsg *imu_msg);

/*!
 * @class LpmsSensor
 * @brief LpmsSensor inherits from IMUCollector
 *
 * it is a collector for lpms sensors
 * if you want more details, refer to header files in lpsensor folder
 * or the site : https://www.lp-research.com/
 */
class LpmsSensor : public IMUCollector
{
private:
  LpmsSensorManagerI *manager;
  LpmsSensorI *lpms;

public:
  LpmsSensor(){};
  ~LpmsSensor() { PRINT_INFO("destroy a lpms imu!"); };

  void init(char path[]) override;

  /*!
   * @brief
   *
   * we use the lpms sensors in the stream mode
   * it is a mode that we do not need to send any command
   * we just set a callback function and when the data arrives,
   * the function will be called
   */
  void set_callback(LpmsCallback lpms_callback);
  void deInit() override;
  int32_t get_imu_msg(Bgs::ImuMsg *imu_msg) override;
};
}

// eof