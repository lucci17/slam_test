#include "udp_socket_client.h"
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string.h>

UdpSocketServer::UdpSocketServer()
    : m_open_succeed(false)
    , m_socket_fd(0)
    , m_port_num(2368)
{
  m_socket_fd = socket(AF_INET, SOCK_DGRAM, 0); // AF_INET:IPV4; SOCK_DGRAM:UDP
  if (m_socket_fd < 0) {
    printf("[Liuyc] Create socket fail!\n");
    return;
  }

  memset(&m_server_addr, 0, sizeof(m_server_addr));
  m_server_addr.sin_family = AF_INET;
  m_server_addr.sin_addr.s_addr =
    htonl(INADDR_ANY); // IP地址，需要进行网络序转换，INADDR_ANY：本地地址
  m_server_addr.sin_port = htons(m_port_num); //端口号，需要网络序转换

  if (bind(m_socket_fd, (struct sockaddr *)&m_server_addr,
           sizeof(m_server_addr))
      < 0) {
    printf("socket bind fail!\n");
    return;
  }

  if (fcntl(m_socket_fd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
    printf("socket fcntl fail!\n");
    return;
  }

  m_open_succeed = true;
  printf("[Liuyc]Sueeccd in initializing the udp socket server!\n");
}

UdpSocketServer::~UdpSocketServer()
{
  if (m_socket_fd >= 0)
    close(m_socket_fd);
}

int32_t UdpSocketServer::get_data(char *buf, int32_t size)
{
  if (!m_open_succeed)
    return -1;

  struct pollfd fds[1];
  fds[0].fd = m_socket_fd;
  fds[0].events = POLLIN;
  const int POLL_TIMEOUT = 1000;

  // recvfrom 是堵塞式的，如果没有数据会一直等
  // 所以先用 poll 判断是否有数据再用 recvfrom 来读取数据
  int retval = 0;
  do {
    retval = poll(fds, 1, POLL_TIMEOUT);
    if (retval < 0) // poll() error?
    {
      if (errno != EINTR)
        printf("[Liuyc] poll() error: %s \n", strerror(errno));
      return -1;
    }
    if (retval == 0) // poll() timeout?
    {
      printf("[Liuyc] lslidar poll() timeout \n");
      return -1;
    }
    if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP)
        || (fds[0].revents & POLLNVAL)) // device error?
    {
      printf("[Liuyc] poll() reports lslidar error \n");
      return -1;
    }
  } while ((fds[0].revents & POLLIN) == 0);

  struct sockaddr_in clent_addr; // clent_addr用于记录发送方的地址信息
  socklen_t len = sizeof(clent_addr);
  memset(buf, 0, size);
  int32_t count =
    recvfrom(m_socket_fd, buf, size, 0, (struct sockaddr *)&clent_addr,
             &len); // recvfrom是拥塞函数，没有数据就一直拥塞
  if (count == -1) {
    printf("recieve data fail!\n");
    return -1;
  }
  // printf( "client:%s\n", buf );  //打印client发过来的信息

  return count;
}
