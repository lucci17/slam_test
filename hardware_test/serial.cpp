#include "serial.h"
#include "macro_defines.h"
#include "struct_defines.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <string>

namespace hardware
{

SerialPort::SerialPort(const char *file, int32_t baud_rate_, int32_t data_size_,
                       char parity_, int32_t stop_bit_)
{
  init(file, baud_rate_, data_size_, parity_, stop_bit_);
}

SerialPort::SerialPort(const char *file, int32_t baud_rate_, int32_t data_size_,
                       char parity_, int32_t stop_bit_,
                       const SerialPort::SerialPortCallback &callback)
    : callback_(callback)
{
  init(file, baud_rate_, data_size_, parity_, stop_bit_);
}

int32_t SerialPort::init(const char *file, int32_t baud_rate_,
                         int32_t data_size_, char parity_, int32_t stop_bit_)
{
  serial_port_ = boost::make_shared<boost::asio::serial_port>(io_service_);
  opened_ = false;
  if (!serial_port_)
    return BGS_ERROR;

  boost::system::error_code ec;
  serial_port_->open(file, ec);
  if (ec) {
    PRINT_ERROR_FMT("open serial %s failed : %s", file, ec.message().c_str());
    return BGS_ERROR;
  }
  // baud rate
  serial_port_->set_option(boost::asio::serial_port::baud_rate(baud_rate_));
  // flow control
  serial_port_->set_option(boost::asio::serial_port::flow_control(
    boost::asio::serial_port_base::flow_control::none));
  // parity
  switch (parity_) {
  case 'O':
    serial_port_->set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::odd));
    break;
  case 'E':
    serial_port_->set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::even));
    break;
  case 'N':
    serial_port_->set_option(
      boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    break;
  default:
    perror("Incorrect parity ");
    return BGS_ERROR;
  }

  // stop bits
  if (stop_bit_ == 1) {
    serial_port_->set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::one));
  } else if (stop_bit_ == 2) {
    serial_port_->set_option(boost::asio::serial_port::stop_bits(
      boost::asio::serial_port::stop_bits::two));
  }

  // data size
  serial_port_->set_option(
    boost::asio::serial_port::character_size(data_size_));
  if (ec) {
    std::cout << "failed to open serial: " << ec.message() << std::endl;
    return BGS_ERROR;
  }

  work_ = boost::make_shared<boost::asio::io_service::work>(io_service_);
  run_thread_ =
    boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));

  // io_service_.run();
  opened_ = true;
  return BGS_OK;
}

SerialPort::~SerialPort()
{
  if (work_)
    work_.reset();

  if (serial_port_) {
    serial_port_->cancel();
    serial_port_->close();
    serial_port_.reset();
  }

  io_service_.stop();
  io_service_.reset();

  while (!run_thread_.joinable())
    Bgs::msleep(10);
  run_thread_.join();
}

int32_t SerialPort::write(char *message, int32_t size)
{
  boost::system::error_code ec;
  if (opened_)
    serial_port_->write_some(boost::asio::buffer(message, size), ec);

  if (ec) {
    PRINT_ERROR_FMT("serial write failed: %s", ec.message().c_str());
    return BGS_ERROR;
  }

  return BGS_OK;
}

int32_t SerialPort::read(char *message, int32_t size)
{
  boost::system::error_code ec;
  int32_t read_size = 0;
  if (opened_)
    read_size = serial_port_->read_some(boost::asio::buffer(message, size), ec);

  if (ec || read_size == 0) {
    PRINT_ERROR_FMT("serial read failed: %s", ec.message().c_str());
    return BGS_ERROR;
  }

  return read_size;
}

void SerialPort::async_read()
{
  if (!serial_port_ || !serial_port_->is_open()) {
    PRINT_ERROR("the port is closed");
    return;
  }

  serial_port_->async_read_some(
    boost::asio::buffer(read_buf_raw_, sizeof(read_buf_raw_)),
    boost::bind(&SerialPort::on_recieve_, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
}

void SerialPort::on_recieve_(const boost::system::error_code &ec,
                             size_t bytes_transferred)
{
  if (ec) {
    // PRINT_ERROR_FMT("async read some error : %s", ec.message().c_str() );
    return;
  }

  if (bytes_transferred > 0) {
    callback_(read_buf_raw_, bytes_transferred);
  } else {
    PRINT_ERROR("recieved nothing!");
  }

  async_read();
}

bool SerialPort::is_running()
{
  if (serial_port_)
    return opened_ && serial_port_->is_open();

  return opened_;
}

} // namespace Bgs
