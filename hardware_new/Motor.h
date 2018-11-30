/*!
 * @file Motor.h
 * @brief Head file for class Motor, GeekMotor, DoubleGeekMotor, Maxon.
 *
 * @author zong.zhang.lin
 *         edward.liu
 * @date 2018-10-08
 */
#pragma once

#include "macro_defines.h"
#include "serial.h"
#include "stdio.h"
#include "struct_defines.h"
#include <stdint.h>

enum MotorType { kMaxon, kGeek, kDoubleGeek, kCopley, kMotorTypeCount };

namespace hardware
{

#define MOTOR_OFF -1
#define MOTOR_ON 0

/*!
 * @brief class Motor
 *
 * It is a interface for all kinds of motors
 * (a virtual class)
 */
class Motor
{
protected:
  int32_t motorFile;
  int16_t leftSpeed;
  int16_t rightSpeed;

  /// \brief factor from rpm(got from motor driver)
  /// to line speed in standard unit(m/s)
  float f_rpm_to_distance;

public:
  Motor()
      : motorFile(MOTOR_OFF)
      , leftSpeed(0)
      , rightSpeed(0)
      , f_rpm_to_distance(1.)
  {
  }

  ~Motor(){};

  /*!
   * @brief initialisation with only one device path
   */
  virtual int initialisation(const char *interfacePath)
  {
    (void)interfacePath;
    return 0;
  }

  /*!
   * @brief initialisation with 2 device paths
   */
  virtual int initialisation(const char *, const char *) { return 0; }

  /*!
   * @brief set speed of both sides
   *
   * usually, if we want to control a robot, we need
   * to control at least 2 motors on both sides.
   * we can set the speed of motors on both sides with
   * this function
   *
   * every kind of motor has its own implemetation
   */
  virtual int setSpeed(int16_t leftSpeed, int16_t rightSpeed)
  {
    (void)leftSpeed;
    (void)rightSpeed;
    return 0;
  }

  /*!
   * @brief get speed of both sides
   *
   * every kind of motor has its own implemetation
   */
  virtual int getSpeed(int16_t &leftSpeed, int16_t &rightSpeed)
  {
    (void)leftSpeed;
    (void)rightSpeed;
    return 0;
  }

  int set_rpm_to_distance(float rpmToDis)
  {
    f_rpm_to_distance = rpmToDis;
    return 0;
  }

  /*!
   * @brief should be called before deleting the instance
   */
  virtual void deInit() {}
};

/*!
 * @brief class GeekMotor for geek driver
 *
 * we can use it to control 2 motors with only 1 driver.
 * so initialising with one driver path.
 */
class GeekMotor : public Motor
{

public:
  GeekMotor();
  ~GeekMotor() { PRINT_INFO("destroy a geek motor."); }

  int initialisation(const char *interfacePath) override;

  int setSpeed(int16_t leftSpeed, int16_t rightSpeed) override;

  int getSpeed(int16_t &leftSpeed, int16_t &rightSpeed) override;

  void deInit() override;

private:
  boost::shared_ptr<SerialPort> serial_port_;
};

/*!
 * @brief class CopleyMotor for 2 Copley drivers
 *
 * send the command to master driver and it will control the slave one
 * so initialising with one driver path.
 */
class CopleyMotor : public Motor
{
public:
  enum MotorLocation { kLeft, kRight, kMotorLocCount };

  CopleyMotor()
      : Motor()
  {
  }
  ~CopleyMotor() {}

  int32_t initialisation(const char *interfacePath);
  int32_t setMode(int mode);
  int32_t setSpeed(int16_t leftSpeed, int16_t rightSpeed) override;
  int32_t getSpeed(int16_t &leftSpeed, int16_t &rightSpeed) override;
  void deInit() override;

private:
  boost::shared_ptr<SerialPort> serial_port_;
};

/*!
 * @brief class DoubleGeekMotor for 2 geek drivers
 *
 * we can use it to control 4 motors with 2 drivers.
 * so initialising with 2 driver paths.
 */
class DoubleGeekMotor : public Motor
{
public:
  enum MotorLocation { kLeft, kRight, kMotorLocCount };

  DoubleGeekMotor()
      : Motor()
  {
  }
  ~DoubleGeekMotor() {}

  int32_t initialisation(const char *interfacePath1,
                         const char *interfacePath2);
  int32_t setSpeed(int16_t leftSpeed, int16_t rightSpeed) override;
  int32_t getSpeed(int16_t &leftSpeed, int16_t &rightSpeed) override;
  void deInit() override;

private:
  boost::shared_ptr<SerialPort> serial_ports_[kMotorLocCount];
};

/*!
 * @brief class Maxon for maxon driver
 *
 * this driver class communicats with an i2c device
 * in kernel, details are in bongos repository
 */
class Maxon : public Motor
{
private:
  unsigned char leftState, rightState;

public:
  Maxon();
  ~Maxon() {}

  int initialisation(const char *interfacePath);

  int setSpeed(int16_t leftSpeed, int16_t rightSpeed);

  int getSpeed(int16_t &leftSpeed, int16_t &rightSpeed);

  void deInit();
};

} // namespace hardware
