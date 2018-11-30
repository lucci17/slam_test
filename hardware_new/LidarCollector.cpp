/*************************************************
* Copyright (c) 2017, Shanghai Bongostech Co.,Ltd.
* All rights reserved.
*
* ------------------------------------------------
*
* This file contains the defination of Obstacle classes.
*/

#include "LidarCollector.h"
/* M_PI/180.0 */
#define BGS_LIDAR_DEG2RAD 0.01745329251

using namespace Bgs;
using namespace hardware::LASER;


namespace hardware
{
const int32_t max_error_count = 2;

int32_t LMS1Collector::Init(char host[], int32_t port)
{
  // laser data
  LASER::scanDataCfg dataCfg;

  // try 20 times
  for (int i = 0; i < 2; ++i) {
    if (i != 0)
      PRINT_INFO("Try one more time!");

    _laser.connect(host, port);
    if (!_laser.isConnected()) {
      sleep(1);
      continue;
    }

    PRINT_INFO("Logging in to laser.");
    _laser.login();
    _cfg = _laser.getScanCfg();
    _outputRange = _laser.getScanOutputRange();

    if (_cfg.scaningFrequency != 5000) {
      _laser.disconnect();
      sleep(1);
      continue;
    }

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    PRINT_INFO("Setting scan data configuration.");
    _laser.setScanDataCfg(dataCfg);

    PRINT_INFO("Starting measurements.");
    _laser.startMeas();

    status_t stat = _laser.queryStatus();
    sleep(1);
    if (stat != ready_for_measurement) {
      PRINT_WARNING("Laser not ready. Retrying initialization.");
      _laser.disconnect();
      sleep(1);
      continue;
    }

    PRINT_INFO("Starting device.");
    _laser.startDevice(); // Log out to properly re-enable system after config

    PRINT_INFO("Commanding continuous measurements.");
    _laser.scanContinous(1);

    GetScanCfg(_laser_scan_.get());
    _init_finished_ = true;
    break;
  }
  if (_init_finished_) {
    PRINT_DEBUG("lms1xx lidar finished initialization!");
  }

  return (_init_finished_ ? BGS_OK : BGS_ERROR);
}

void LMS1Collector::DeInit()
{
  _exit_thread_ = true;
  do {
    Bgs::msleep(10);
  } while (!_thread_update_data_.joinable());
  _thread_update_data_.join();
  // This still not work
  if (_laser.isConnected()) {
    // 			laser.scanContinous(0);
    // 			laser.stopMeas();
    _laser.disconnect();
  }
}

int32_t LMS1Collector::GetScanMsg(Bgs::LaserScanMsg *scan_msg)
{
  scan_msg->header.stamp = BgsTime::get_current_time();
  if (_laser.getScanData(&_data)) {
    scan_msg->laser_scan_count = _data.dist_len1;
    for (int32_t i = 0; i < (int32_t)_data.dist_len1; i++) {
      // 这里注释掉了对于反射率的判断
      // 因为有时候会出现反射率都是0的情况，目前还不明原因，需要调查 TODO
      scan_msg->intensities[i] = _data.rssi1[i] * 0.001f;
      scan_msg->ranges[i] = _data.dist1[i] * 0.001f;
    }

    return BGS_OK;
  } else {
    PRINT_ERROR(
      "Laser timed out on delivering scan, attempting to reinitialize.");
    return BGS_ERROR;
  }
}

int32_t LMS1Collector::GetScanCfg(Bgs::LaserScanMsg *scan_msg)
{
  scan_msg->range_min = 0.01;
  scan_msg->range_max = 35.0;

  scan_msg->scan_time = 100.0 / _cfg.scaningFrequency;
  scan_msg->angle_increment =
    (double)_outputRange.angleResolution / 10000.0 * BGS_LIDAR_DEG2RAD;
  scan_msg->angle_min =
    (double)_outputRange.startAngle / 10000.0 * BGS_LIDAR_DEG2RAD - M_PI_2;
  scan_msg->angle_max =
    (double)_outputRange.stopAngle / 10000.0 * BGS_LIDAR_DEG2RAD - M_PI_2;
  scan_msg->time_increment = (_outputRange.angleResolution / 10000.0) / 360.
                             / (_cfg.scaningFrequency / 100.0);

  PRINT_DEBUG_FMT("scan range: %f, %f, angle increment: %f",
                  scan_msg->angle_min, scan_msg->angle_max,
                  scan_msg->angle_increment);

  return BGS_OK;
}

void LMS1Collector::UpdateData()
{
  int32_t error_count = 0;
  while (!_exit_thread_) {
    if (!_init_finished_) {
      msleep(10);
      continue;
    }

    if (GetScanMsg(_laser_scan_.get()) == BGS_OK) {
      int32_t ret = _callback_();
      if (ret > 0) // 并没有报错，但是忽略这一次的操作
      {
        msleep(10);
        continue;
      } else if (ret == BGS_OK) {

      } else if (ret < 0)
        error_count++;
    } else
      error_count++;

    if (error_count > max_error_count) {
      PRINT_ERROR(
        "failed to get scan msg for several times. inner thread exit!");
      on_error_();
      break;
    } else {
      Bgs::msleep(10);
      continue;
    }
  }

  PRINT_INFO("lmx1xx thread exit.");
}

int32_t LMS1Collector::GetPointCloud2Cfg(Bgs::PointCloud2Ptr &cloud)
{
  PRINT_ERROR(
    "It is a 2d laser, and connot provide you with a PointCloudMsg! ");
  return BGS_ERROR;
}

int32_t LMS1Collector::GetPointCloud2(Bgs::PointCloud2Ptr &cloud)
{
  PRINT_ERROR(
    "It is a 2d laser, and connot provide you with a PointCloudMsg! ");
  return BGS_ERROR;
}

int32_t RoboSenseCollector::Init(char host[], int32_t port)
{
  (void)host;
  (void)port;

  return BGS_OK;
}

void RoboSenseCollector::DeInit()
{
  _exit_thread_ = true;
  do {
    Bgs::msleep(10);
  } while (!_thread_update_data_.joinable());
  _thread_update_data_.join();
}

int32_t RoboSenseCollector::GetScanMsg(Bgs::LaserScanMsg *scan_msg)
{
  PRINT_WARNING("it is a 3D lidar, cannot get a scan msg!");
  return BGS_ERROR;
}
int32_t RoboSenseCollector::GetScanCfg(Bgs::LaserScanMsg *scan_msg)
{
  PRINT_WARNING("it is a 3D lidar, cannot get a scan msg!");
  return BGS_ERROR;
}

int32_t RoboSenseCollector::GetPointCloud2Cfg(Bgs::PointCloud2Ptr &cloud)
{
  return BGS_OK;
}

int32_t RoboSenseCollector::GetPointCloud2(Bgs::PointCloud2Ptr &cloud)
{
  if (_lidar_ && _convertor_) {
    RslidarScanPtr scan(new RslidarScan);
    if (_lidar_->poll(scan))
      _convertor_->processScan(scan, cloud);
    else
      return BGS_ERROR;
  } else
    return BGS_ERROR;

  // 		PRINT_DEBUG_FMT("intensities: %lf", cloud->points[0].intensity );
  return BGS_OK;
}

void RoboSenseCollector::UpdateData()
{
  int32_t error_count = 0;
  // 		const int32_t max_error_count = 5;
  while (!_exit_thread_) {
    if (!_init_finished_) {
      msleep(10);
      continue;
    }
    if (GetPointCloud2(_point_cloud2_) == BGS_OK) {
      int32_t ret = _callback_();
      if (ret > 0) // 并没有报错，但是忽略这一次的操作
      {
        msleep(10);
        continue;
      } else if (ret == BGS_OK) {

      } else if (ret < 0)
        error_count++;
    } else
      error_count++;

    if (error_count > max_error_count) {
      PRINT_ERROR(
        "failed to get point cloud msg for several times. inner thread exit!");
      on_error_();
      break;
    } else {
      Bgs::msleep(10);
      continue;
    }
  }

  PRINT_INFO("lidar thread exit.");
}
}