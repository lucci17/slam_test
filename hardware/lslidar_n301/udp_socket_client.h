#ifndef UDP_SOCKET_CLIENT_H
#define UDP_SOCKET_CLIENT_H

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class UdpSocketServer
{
public:
  UdpSocketServer();
  ~UdpSocketServer();

  int32_t get_data(char *buf, int32_t size);

private:
  bool m_open_succeed;
  int32_t m_socket_fd;
  int32_t m_port_num;
  struct sockaddr_in m_server_addr;
};

#endif
