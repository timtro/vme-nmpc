/*
 * vme-nmpc/lib/VmeNmpcWin.cpp
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

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>

#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

VmeNmpc::VmeNmpc()
    : hostname_{"localhost"},
      portno_{5010},
      sockfd{},
      is_connected_{false} {
      // Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
  #pragma comment (lib, "Ws2_32.lib")
  #pragma comment (lib, "Mswsock.lib")
  #pragma comment (lib, "AdvApi32.lib")
}

VmeNmpc::VmeNmpc(std::string host, unsigned int portno)
    : hostname_{std::move(host)},
      portno_{portno},
      sockfd{},
      is_connected_{false} {}

VmeNmpc::~VmeNmpc() {
  WSACleanup();
  closesocket(sockfd);
}

void VmeNmpc::connect() {
  // TODO(TT): Create a timeout exception. Temporarily set the socket to non-
  // blocking and use select() to implement a timeout.
  if (is_connected_)
    throw std::logic_error(
        "Connection attempt while already connected to server."
        " Disconnect before attempting a new connection.");

    WSADATA wsaData;
    SOCKET sockfd = INVALID_SOCKET;
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    char *sendbuf = "this is a test";
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(hostname_, portno_, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Attempt to connect to an address until one succeeds
    for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

        // Create a SOCKET for connecting to server
        sockfd = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (sockfd == INVALID_SOCKET) {
            printf("socket failed with error: %ld\n", WSAGetLastError());
            WSACleanup();
            return 1;
        }

        // Connect to server.
        iResult = connect( sockfd, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(sockfd);
            sockfd = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (sockfd == INVALID_SOCKET) {
        printf("Unable to connect to server!\n");
        WSACleanup();
        return 1;
    }

}

/**
 * Safely disconnect from the VmeNmpc machine.
 */
void VmeNmpc::disconnect() {
  clear_targets();
  closesocket(sockfd);
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

// Wrapped CLI commands

int VmeNmpc::at(float x, float y, float tol) {
  std::string msg = "AT " + std::to_string(x) + ' ' + std::to_string(y) + ' ' +
                    std::to_string(tol) + '\n';
  return sendstr(msg);
}

int VmeNmpc::clear_targets() { return sendstr("\nCLEAR ALL\n"); }
