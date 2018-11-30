/*************************************************
* Copyright (c) 2017, Shanghai Bongostech Co.,Ltd.
* All rights reserved.
*
* ------------------------------------------------
*
* This file contains the defination of Obstacle classes.
*/

#include "IMUCollector.h"

#ifndef GRAVITY_CONSTANT
#define GRAVITY_CONSTANT 10.
#endif

namespace hardware
{

void LpmsSensor::init(char path[])
{
  manager = LpmsSensorManagerFactory();
  manager->setVerbose(false);
  char ch_dev_name[64];
  memset(ch_dev_name, 0, sizeof(ch_dev_name));
  if (path != NULL && *path != '\0') {
    strcpy(ch_dev_name, path);
  }

  if (ch_dev_name[0] == '\0') {
    strcpy(ch_dev_name, "/dev/ttyUSB0");
  }
  PRINT_INFO_FMT("LPMS Device: %s", ch_dev_name);
  lpms = manager->addSensor(DEVICE_LPMS_RS232, ch_dev_name);

  // init
  // lpms 的初始化是在另一个进程里面进行的，所以这里需要循环等待 lpms
  // 初始化全部完成才能进行下一步
  // 这里最长等待 60s，
  // 若60秒等待不成功则直接跳过，会有一条打印消息但并不返回错误
  bool b_init_error = true;
  for (int i = 0; i < 60; ++i) {
    sleep(1);
    if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED
        && lpms->hasImuData()) {
      PRINT_INFO("lpms initialization has finished!");
      b_init_error = false;
      break;
    }
  }
  if (b_init_error) {
    PRINT_ERROR("lpms init out of time!");
  }

  // change the mode to stream
  PRINT_INFO("Change mode to stream");
  lpms->setConfigurationPrm(PRM_LPBUS_DATA_MODE, SELECT_LPMS_MODE_STREAM);
  for (int i = 0; i < 60; ++i) {
    sleep(1);
    if (lpms->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED
        && lpms->hasImuData()) {
      PRINT_INFO("lpms changed to stream mode!");
      b_init_error = false;
      break;
    }
  }
  if (b_init_error) {
    PRINT_ERROR("lpms init out of time!");
  }
}

void LpmsSensor::deInit()
{
  if (manager) {
    // Removes the initialized sensor
    manager->removeSensor(lpms);

    // Deletes LpmsSensorManager object
    delete manager;
    manager = NULL;
  }
}

void ImuData_to_ImuMsg(const ImuData &data, Bgs::ImuMsg *imu_msg)
{
  imu_msg->header.stamp = BgsTime::get_current_time();
  imu_msg->header.frame_id = "imu_link";

  imu_msg->orientation.w = data.q[0] /*- yaw_offset.w*/;
  imu_msg->orientation.x = data.q[1] /*- yaw_offset.x*/;
  imu_msg->orientation.y = data.q[2] /*- yaw_offset.y*/;
  imu_msg->orientation.z = data.q[3] /*- yaw_offset.z*/;

  // modified on 2018.07.20
  // collect the raw data instead of claculated data
  /*
  imu_msg->linear_acceleration.x = -data.a[0] * GRAVITY_CONSTANT;
  imu_msg->linear_acceleration.y = -data.a[1] * GRAVITY_CONSTANT;
  imu_msg->linear_acceleration.z = -data.a[2] * GRAVITY_CONSTANT;
  */
  imu_msg->linear_acceleration.x = -data.aRaw[0] * GRAVITY_CONSTANT;
  imu_msg->linear_acceleration.y = -data.aRaw[1] * GRAVITY_CONSTANT;
  imu_msg->linear_acceleration.z = -data.aRaw[2] * GRAVITY_CONSTANT;

  const double trans = M_PI / 180.;
  /*
  imu_msg->angular_velocity.x = data.g[0] * trans;
  imu_msg->angular_velocity.y = data.g[1] * trans;
  imu_msg->angular_velocity.z = data.g[2] * trans;
  */
  imu_msg->angular_velocity.x = data.gRaw[0] * trans;
  imu_msg->angular_velocity.y = data.gRaw[1] * trans;
  imu_msg->angular_velocity.z = data.gRaw[2] * trans;

  // 		std::cout << "acc: " << imu_msg->linear_acceleration.x << " "
  // 							<< imu_msg->linear_acceleration.y << " "
  // 							<< imu_msg->linear_acceleration.z << " "
  // 				  << " angular_velocity: " << imu_msg->angular_velocity.x << " "
  // 				  << imu_msg->angular_velocity.y << " " <<
  // imu_msg->angular_velocity.z
  // << std::endl;
  imu_msg->roll = data.r[0] * trans;
  imu_msg->pitch = data.r[1] * trans;
  imu_msg->yaw = data.r[2] * trans;
}

int32_t LpmsSensor::get_imu_msg(Bgs::ImuMsg *imu_msg)
{
  if (lpms->getConnectionStatus() != SENSOR_CONNECTION_CONNECTED)
    return BGS_ERROR;

  ImuData data;
  lpms->update();
  const int32_t time_out = 1000;
  int32_t time_index = 0;
  while (!lpms->hasImuData()) {
    usleep(1000);
    time_index++;
    if (time_index > time_out) {
      PRINT_ERROR("Failed to get imu data!");
      return BGS_ERROR;
    }
  }
  data = lpms->getCurrentData();
  ImuData_to_ImuMsg(data, imu_msg);

  return BGS_OK;
}

void LpmsSensor::set_callback(LpmsCallback lpms_callback)
{
  lpms->setCallback(lpms_callback);
}
}