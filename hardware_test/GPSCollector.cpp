#include "GPSCollector.h"

namespace hardware
{
// Init serial port
int32_t GPSCollector::init(char *path)
{
  SerialPort::SerialPortCallback callback = [this](char *raw_data_buffer,
                                                   int32_t length) {
    raw_data_buffer[length] = '\0';
    int32_t read_flag = solution_.readByte(raw_data_buffer, length);

    // 返回 0 说明已经获得了完整的一帧 GPS 数据
    if (read_flag == 0) {
      gnss::GPSGesture current_gst = solution_.getGestureData();
      gnss::GPSData current_gps = solution_.getPositionData();

      if (has_callback_)
        callback_(current_gps, current_gst);
      else
        PRINT_DEBUG("there is no callback function");
    }
  };

  serial_port_ =
    boost::make_shared<SerialPort>(path, 115200, 8, 'N', 1, callback);
  if (serial_port_ && serial_port_->is_running())
    PRINT_INFO("GPS initialisation succeeded!");
  else {
    PRINT_ERROR("GPS init failed.");
    return BGS_ERROR;
  }

  serial_port_->async_read();
  return BGS_OK;
}

// Init serial port
void GPSCollector::deInit()
{
  // 关闭串口
  if (serial_port_)
    serial_port_.reset();
}

void GPSCollector::set_callback(const GPSCollector::GpsCallback &callback)
{
  callback_ = callback;
  has_callback_ = true;
}
}