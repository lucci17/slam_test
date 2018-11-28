#ifndef N301_DRIVER_H
#define N301_DRIVER_H

#include "../lidar_driver.h"
#include "n301_decoder.h"
#include "n301_struct.h"
#include "udp_socket_client.h"

static uint16_t UDP_PORT_NUMBER = 2368;
static uint16_t PACKET_SIZE = 1206;

class N301_decoder;
class UdpSocketServer;
class N301_driver : public UdpSocketServer, public LidarDriver
{
public:
  static N301_driver *instance();
  N301_driver();
  ~N301_driver();

  bool initialize();
  void lidar_data_update();
  bool get_sweep_data(N301_lidar *);

private:
  N301_decoder *m_n301_decoder;
};

#endif // N301_DRIVER_H
