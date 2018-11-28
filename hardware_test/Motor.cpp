#include "Motor.h"

namespace hardware
{

GeekMotor::GeekMotor()
    : Motor()
{
}

int32_t GeekMotor::initialisation(const char *interfacePath)
{
  serial_port_ =
    boost::make_shared<SerialPort>(interfacePath, 115200, 8, 'N', 1);
  if (serial_port_ && serial_port_->is_running())
    PRINT_INFO("Motor initialisation succeeded!");
  else {
    PRINT_ERROR("Motor init failed.");
    return -1;
  }
  return 0;
}

int GeekMotor::setSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
  int32_t sizeOfChar = 0;
  char command[64] = { '\0' };
  if (!serial_port_->is_running()) {
    PRINT_WARNING("motor removed");
    return BGS_ERROR;
  }

  // due to the internal controller send a inverse value to the motor
  // there we need to send a inverse value to internal controller
  sizeOfChar = sprintf(command, "!M %d %d\r", -leftSpeed, rightSpeed);
  // printf("%s \n",command);

  if (serial_port_->write(command, sizeOfChar) == BGS_ERROR) {
    PRINT_WARNING("fail to send command to motor");
    return BGS_ERROR;
  }

  return BGS_OK;
}

int GeekMotor::getSpeed(int16_t &leftSpeedIn, int16_t &rightSpeedIn)
{
  std::string response;
  int32_t pos = 0, pos2 = 0;
  if (!serial_port_->is_running()) {
    PRINT_WARNING("motor removed");
    return BGS_ERROR;
  }

  std::string get_speed_cmd = "?S\r";
  if (serial_port_->write(const_cast<char *>(get_speed_cmd.c_str()), 3)
      == BGS_ERROR) {
    PRINT_WARNING("fail to send command to motor");
    return BGS_ERROR;
  }

  int ret = 0;
  char buffer[64] = { '\0' };
  memset(buffer, 0, sizeof(buffer));
  ret = serial_port_->read(buffer, 64);
  // buffer[ret+1] ='\0';
  if (ret <= 0) {
    PRINT_ERROR("Got nothing from the port");
    return BGS_ERROR;
  }

  response.assign(buffer, ret);
  pos = response.find("=");
  if (pos == -1) {
    leftSpeedIn = 0;
    rightSpeedIn = 0;
    return BGS_ERROR;
  }
  pos2 = response.find(":");
  std::string leftSpeed = response.substr(pos + 1, pos2 - pos - 1);
  std::string rightSpeed =
    response.substr(pos2 + 1, response.length() - pos2 - 1);
  // printf("%d %d
  // \n",std::atoi(leftSpeed.c_str()),std::atoi(rightSpeed.c_str()));
  leftSpeedIn = -(int)(std::atoi(leftSpeed.c_str()) * f_rpm_to_distance);
  rightSpeedIn = (int)(std::atoi(rightSpeed.c_str()) * f_rpm_to_distance);

  // 		if((leftSpeedIn>0&&rightSpeedIn>0)||(leftSpeedIn<0&&rightSpeedIn<0))
  // 		{
  // 			leftSpeedIn=-leftSpeedIn;
  // 			rightSpeedIn=-rightSpeedIn;
  // 		}

  return ret;
}

void GeekMotor::deInit() {}

// TODO change into SerialPort class
int32_t DoubleGeekMotor::initialisation(const char *interfacePathLeft,
                                        const char *interfacePathRight)
{
  const char *interfacepath[kMotorLocCount] = { interfacePathLeft,
                                                interfacePathRight };
  for (int i = 0; i < kMotorLocCount; ++i) {
    serial_ports_[i] =
      boost::make_shared<SerialPort>(interfacepath[i], 115200, 8, 'N', 1);
    if (serial_ports_[i] && serial_ports_[i]->is_running())
      continue;

    PRINT_ERROR("Motor init failed.");
    return BGS_ERROR;
  }

  PRINT_INFO("Motor initialisation succeeded!");
  return BGS_OK;
}

int32_t DoubleGeekMotor::setSpeed(int16_t leftSpeed, int16_t rightSpeed)
{
  if (!serial_ports_[kLeft]->is_running()
      || !serial_ports_[kRight]->is_running()) {
    PRINT_ERROR("cannot set speed!");
    return BGS_ERROR;
  }

  const int16_t n_speed[kMotorLocCount] = { leftSpeed, rightSpeed };
  for (int i = 0; i < kMotorLocCount; ++i) {
    char command[64];
    memset(command, 0, sizeof(command));
    int32_t size_of_command = 0;

    size_of_command = sprintf(command, "!M %d %d\r", 0, -n_speed[i]);
    // PRINT_DEBUG_FMT("command: %s", command);

    if (serial_ports_[i]->write(command, size_of_command) == BGS_ERROR) {
      PRINT_WARNING("fail to send command to motor");
      return BGS_ERROR;
    }

    // m_serials[i].Flush();
  }
  return BGS_OK;
}

int32_t DoubleGeekMotor::getSpeed(int16_t &leftSpeed, int16_t &rightSpeed)
{
  if (!serial_ports_[kLeft]->is_running()
      || !serial_ports_[kRight]->is_running()) {
    PRINT_ERROR("cannot get speed!");
    return BGS_ERROR;
  }

  leftSpeed = rightSpeed = 0;
  int16_t n_speed[kMotorLocCount];
  memset(n_speed, 0, sizeof(n_speed));
  char read_buffer[64] = { '\0' };
  memset(read_buffer, 0, sizeof(read_buffer));
  for (int i = 0; i < kMotorLocCount; ++i) {
    std::string get_speed_cmd = "?S\r";
    if (serial_ports_[i]->write(const_cast<char *>(get_speed_cmd.c_str()), 3)
        == BGS_ERROR) {
      PRINT_WARNING("fail to send command to motor");
      return BGS_ERROR;
    }

    int32_t pos = 0, pos2 = 0;
    std::string response;
    memset(read_buffer, 0, sizeof(read_buffer));
    int32_t ret = serial_ports_[i]->read(read_buffer, 64);
    // int32_t ret = read(m_serials[i].file, buffer, 64);
    if (ret == BGS_ERROR) {
      PRINT_WARNING("fail to read from motor");
      return BGS_ERROR;
    }

    response.assign(read_buffer, ret);
    pos = response.find("=");
    if (pos == -1) {
      PRINT_WARNING("Cannot find \"=\" in return msg ");
      n_speed[i] = 0;
      continue;
    }

    pos2 = response.find(":");
    if (pos2 == -1) {
      PRINT_WARNING("Cannot find \":\" in return msg ");
      n_speed[i] = 0;
      continue;
    }

    std::string str_tmp_speed1 = response.substr(pos + 1, pos2 - pos - 1);
    std::string str_tmp_speed2 =
      response.substr(pos2 + 1, response.length() - pos2 - 1);
    int32_t tmp_speed1 = std::atoi(str_tmp_speed1.c_str()) * f_rpm_to_distance;
    int32_t tmp_speed2 = std::atoi(str_tmp_speed2.c_str()) * f_rpm_to_distance;
    n_speed[i] = (int16_t)((tmp_speed1 + tmp_speed2) >> 1);
  }

  leftSpeed = n_speed[kLeft];
  rightSpeed = -n_speed[kRight];

  return BGS_OK;
}

void DoubleGeekMotor::deInit()
{
  setSpeed(0, 0);
  PRINT_INFO("motor stoped");
  for (int i = 0; i < kMotorLocCount; ++i) {
    if (serial_ports_[i])
      serial_ports_[i].reset();
  }
}

Maxon::Maxon()
    : Motor()
{
}

#define MOTOR_BACKWARD 0x01
#define MOTOR_FORWARD 0x10
#define MOTOR_STOP 0x00

int Maxon::initialisation(const char *interfacePath)
{
  motorFile = open(interfacePath, O_RDWR);
  if (motorFile == MOTOR_OFF) {
    perror("Motor is not accessible\n");
    return -1;
  }

  PRINT_INFO("Motor initialisation succeeded!");
  return 0;
}

int Maxon::setSpeed(int16_t leftSpeed, int16_t rightSpeed)
{

  if (motorFile == MOTOR_OFF) {
    printf("motor removed\n");
    return BGS_ERROR;
  }
  // Speed mapping
  uint8_t buffer[4];
  // printf("Writing speed %d %d \n",leftSpeed, rightSpeed);
  memcpy(buffer, &leftSpeed, 2);
  memcpy(buffer + 2, &rightSpeed, 2);
  return write(motorFile, buffer, 4);
}

int Maxon::getSpeed(int16_t &leftSpeedIn, int16_t &rightSpeedIn)
{
  if (motorFile == MOTOR_OFF) {
    printf("motor removed\n");
    return BGS_ERROR;
  }
  int32_t ret = 0;
  uint8_t buffer[6] = { '\0' };
  uint16_t tmp_buffer[3] = { 0 };

  ret = read(motorFile, buffer, 4);
  memcpy(tmp_buffer, buffer, sizeof(buffer));
  // 下面这样指针直接转会报警告，所以采用了 memcpy 来实现
  // leftSpeedIn = leftSpeed = (*((uint16_t *)buffer)) * f_rpm_to_distance;
  // rightSpeedIn= rightSpeed = (*((uint16_t *)(buffer+2))) *
  // f_rpm_to_distance;
  leftSpeedIn = leftSpeed = tmp_buffer[0] * f_rpm_to_distance;
  rightSpeedIn = rightSpeed = tmp_buffer[1] * f_rpm_to_distance;
  leftState = *((uint8_t *)buffer + 4);
  rightState = *((uint8_t *)buffer + 5);
  // printf(" %d %d %x %x\n", leftSpeedIn, rightSpeedIn, leftState, rightState);

  if (leftState == MOTOR_BACKWARD) {
    leftSpeedIn = -leftSpeedIn;
  }
  if (rightState == MOTOR_BACKWARD) {
    rightSpeedIn = -rightSpeedIn;
  }
  return ret;
}

void Maxon::deInit()
{
  if (motorFile != MOTOR_OFF) {
    setSpeed(0, 0);
    close(motorFile);
    motorFile = -1;
    PRINT_INFO("motor stoped");
  }
}
}