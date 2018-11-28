/*!
 * @file serial.h
 * @brief Head file for class SerialPort.
 *
 * @author edward.liu
 * @date 2018-10-08
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

namespace hardware
{

/*!
 * @class SerialPort
 * @brief class SerialPort
 *
 * implemetation of a serial port
 * with boost asio
 *
 * you can use it in windows, unix...
 * and you can use it in both synchronized and
 * asynchronized modes
 *
 * @author edward.liu
 */
class SerialPort
{

public:
  typedef boost::function<void(char *, int32_t)> SerialPortCallback;

  /*!
   * @brief constructor
   *
   * @param file the device name such as /dev/ttyUSB0 in linux system
   * @param baud_rate_ 115200, 9600 ...
   * @param data_size_ most likely: 8
   * @param parity_ O: odd; E: even; N: none
   * @param stop_bit_ 1 or 2
   *
   * @note if you contruct a serial port using this constructor,
   * you can only use the serial in synchronized mode
   */
  SerialPort(const char *file, int32_t baud_rate_, int32_t data_size_,
             char parity_, int32_t stop_bit_);

  /*!
   * @brief constructor
   *
   * @param file the device name such as /dev/ttyUSB0 in linux system
   * @param baud_rate_ 115200, 9600 ...
   * @param data_size_ most likely: 8
   * @param parity_ O: odd; E: even; N: none
   * @param stop_bit_ 1 or 2
   * @param callback the call back function called
   * when recieving data from the serial
   *
   * @note if you contruct a serial port using this constructor,
   * you can only use the serial in asynchronized mode
   */
  SerialPort(const char *file, int32_t baud_rate_, int32_t data_size_,
             char parity_, int32_t stop_bit_,
             const SerialPortCallback &callback);
  ~SerialPort();

  /*!
   * @brief synchronized write
   *
   * @param message buffer that you want to write to serial
   * @param size the size of the buffer
   *
   * @return succeed or failed (0 or -1)
   */
  int32_t write(char *message, int32_t size);

  /*!
   * @brief synchronized read
   *
   * @param message the buffer to get data from serial
   * @param size the size of the buffer
   *
   * @return
   * - if it is -1, failed
   * - if it is over 0, it is the size you actually
   * read from the serial
   */
  int32_t read(char *message, int32_t size);

  /*! @brief start asynchronized read
   *
   * you can start asynchronized reading
   * and the callback function will be called
   * immediatelly when message recieved
   */
  void async_read();

  /// @brief return the running status ( bool )
  bool is_running();

private:
  boost::asio::io_service io_service_;
  boost::shared_ptr<boost::asio::io_service::work> work_;
  boost::shared_ptr<boost::asio::serial_port> serial_port_;

  void on_recieve_(const boost::system::error_code &ec,
                   size_t bytes_transferred);

  /// @brief inner init function for constructor
  int32_t init(const char *file, int32_t baud_rate_, int32_t data_size_,
               char parity_, int32_t stop_bit_);

  char read_buf_raw_[256];
  bool opened_;

  /*! @brief callback in asynchronized mode
   *
   * if used in asynchronized mode, the callback function should
   * be set in constructor.
   */
  SerialPortCallback callback_;

  /// @brief thread for ( boost asio ) io service
  boost::thread run_thread_;
};

typedef boost::shared_ptr<SerialPort> SerialPortPtr;
typedef boost::shared_ptr<const SerialPort> SerialPortConstPtr;

} /// namespace hardware
