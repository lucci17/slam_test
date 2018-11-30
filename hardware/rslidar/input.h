/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2015, Jack O'Quin
 *	Copyright (C) 2017, Robosense, Tony Zhang
 *
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  Input classes for the RSLIDAR RS-16 3D LIDAR:
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __RSLIDAR_INPUT_H_
#define __RSLIDAR_INPUT_H_

#include "bgs_time.h"
#include "macro_defines.h"
#include "rslidar_msgs.h"
#include "struct_defines.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <pcap.h>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <sys/file.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace hardware::rslidar::msgs;
namespace hardware
{
namespace rslidar
{
namespace driver
{

static uint16_t DATA_PORT_NUMBER = 6699; ///< rslidar default data port on PC

class Input
{
public:
  Input(std::string ip, uint16_t port);

  virtual ~Input() {}

  virtual int getPacket(RslidarPacket *pkt, const double time_offset) = 0;

protected:
  uint16_t port_;
  std::string devip_str_;
};

/** @brief Live rslidar input from socket. */
class InputSocket : public Input
{
public:
  InputSocket(std::string ip, uint16_t port = DATA_PORT_NUMBER);

  virtual ~InputSocket();

  virtual int getPacket(RslidarPacket *pkt, const double time_offset);

private:
  int sockfd_;
  in_addr devip_;

  int Ret;
  int len;
};
}
}
}

#endif // __RSLIDAR_INPUT_H
