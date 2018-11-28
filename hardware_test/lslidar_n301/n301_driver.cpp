#include "n301_driver.h"

N301_driver *N301_driver::instance()
{
  static N301_driver obj;
  return &obj;
}

N301_driver::N301_driver()
    : UdpSocketServer()
    , LidarDriver(kLslidarN301)
{
  m_n301_decoder = N301_decoder::instance();
}

N301_driver::~N301_driver() {}

bool N301_driver::initialize() { return true; }

void N301_driver::lidar_data_update()
{
  uint16_t port;
  int32_t recvSize = get_data(m_n301_decoder->msg->data, PACKET_SIZE);
  if (recvSize < 0) // 返回值小于 0 则获取数据时出错，直接返回，不需要再分析
    return;

  if (recvSize == PACKET_SIZE) {
    m_n301_decoder->getOnePacket();
  } else {
    printf("recvSize != PACKET_SIZE\n");
  }
}

bool N301_driver::get_sweep_data(N301_lidar *lidar)
{
  if (lidar == NULL)
    return false;

  lidar_data_update();
  if (m_n301_decoder->isDataReady()) {
    *lidar = *m_n301_decoder->ready_data;
    m_n301_decoder->setDataReady(false);
    return true;
  }

  return false;
}
