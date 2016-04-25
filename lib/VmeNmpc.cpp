/*
 * vme-nmpc/lib/VmeNmpc.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2016-04
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2016 by Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#include "VmeNmpc.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <tuple>
#include <vector>

#include <errno.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

VmeNmpc::VmeNmpc()
    : hostname_{"localhost"},
      portno_{5010},
      sockfd{},
      is_connected_{false} {}

VmeNmpc::VmeNmpc(std::string host, unsigned int portno)
    : hostname_{std::move(host)},
      portno_{portno},
      sockfd{},
      is_connected_{false} {}

VmeNmpc::~VmeNmpc() { close(sockfd); }

void VmeNmpc::connect() {
  // TODO(TT): Create a timeout exception. Temporarily set the socket to non-
  // blocking and use select() to implement a timeout.
  if (is_connected_)
    throw std::logic_error(
        "Connection attempt while already connected to server."
        " Disconnect before attempting a new connection.");

  struct addrinfo hints;
  memset(&hints, 0, sizeof hints);
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo* servinfo;
  {
    // localize scope for rv
    int rv = getaddrinfo(hostname_.c_str(), std::to_string(portno_).c_str(),
                         &hints, &servinfo);

    if (rv != 0) {
      freeaddrinfo(servinfo);
      throw std::runtime_error(
          std::string{"Function getaddrinfo() returned error: "} +
          gai_strerror(rv));
    }
  }

  // loop through serverinfo list.
  struct addrinfo* p;

  for (p = servinfo; p != nullptr; p = p->ai_next) {
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      freeaddrinfo(servinfo);
      throw std::runtime_error(
          std::string{"Function socket() returned error: "} + strerror(errno));
    }

    if (::connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      continue;
    }

    break;  // successfully connected
  }

  if (p == nullptr) {
    freeaddrinfo(servinfo);
    throw std::runtime_error(
        std::string{"Function connect() returned error: "} + strerror(errno));
  }

  freeaddrinfo(servinfo);
  is_connected_ = true;
  return;
}

/**
 * Safely disconnect from the VmeNmpc machine.
 */
void VmeNmpc::disconnect() {
  stop();
  close(sockfd);
  is_connected_ = false;
}

/**
 * Send msg across the TCP/IP connection to the VmeNmpc machine.
 * @param  msg The message to be sent.
 * @return     Forwards the return value from write().
 */
int VmeNmpc::sendstr(std::string msg) {
  return send(sockfd, msg.c_str(), msg.length(), 0);
}

/**
 * Takes a command to be sent to the Nav2 machine, adds a new line to the end
 * if it isn't already there and sends it. This function doesn't do a lot of
 * checking, so if there is a preexisting \n, it has to be the last character
 * in msg, otherwise, it isn't detected.
 * @param  msg The message to be sent.
 */
int VmeNmpc::sendline(std::string cmd) {
  if (cmd.back() == '\n')
    return sendstr(cmd);
  else
    return sendstr(cmd + '\n');
}

void VmeNmpc::set_host(std::string host) {
  if (is_connected_)
    throw std::logic_error("Cannot change VmeNmpc hostname while connected");
  else
    hostname_ = host;
}

void VmeNmpc::set_port(int portno) {
  if (is_connected_)
    throw std::logic_error("Cannot change VmeNmpc port while connected");
  else
    portno_ = portno;
}

/**
 * For two-way communication with the Nav2 machine, this function sends msg to
 * the Nav2 device and waits to recieve a response.
 * @param  msg         The message to send to the Nav2 machine.
 * @param  buffer_size Optionally, you can specify a buffer size. The default
 * is only 128, so you can bump it up to decrease the turns through recieving
 * loop.
 * @return             Returs the string recieved from the Nav2 machine.
 */
std::string VmeNmpc::send_recv(std::string msg, int buffer_size) {
  // const int buffer_size = ;

  if (sendstr(msg) < 0)
    throw std::runtime_error(
        "Failed to write to Nav2 machine during send_recv. msg: " + msg);

  std::vector<char> buffer(buffer_size);
  std::string recieved_message;
  int bytes_received{0};

  do {
    bytes_received = recv(sockfd, buffer.data(), buffer.size(), 0);

    if (bytes_received < 0)
      throw std::runtime_error(
          "Failed to read from Nav2 machine during pose update. msg: " + msg);
    else
      recieved_message.append(buffer.cbegin(), buffer.cend());
  } while (bytes_received == buffer_size);

  return recieved_message;
}
