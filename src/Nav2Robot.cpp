/*
 * vme-nmpc/src/Nav2Robot.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-17
 *
 * This file is part of vme-nmpc
 *
 * Copyright (C) 2015 by Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Nav2Robot.hpp"
#include "trig.h"

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

Nav2Robot::Nav2Robot()   : hostname_{"localhost"}, portno_{5010}, sockfd_{},
  pose_{}, is_connected_{false} {}

Nav2Robot::Nav2Robot(std::string host, unsigned int portno)
  : hostname_{host}, portno_{portno}, sockfd_{}, pose_{}, is_connected_{false} {}

Nav2Robot::~Nav2Robot() {
  close(sockfd_);
}

void Nav2Robot::connect() {
  // TODO(TT): Create a timeout exception. Temporarily set the socket to non-
  // blocking and use select() to implement a timeout.
  if (is_connected_) throw
    std::logic_error("Connection attempt while already connected to server."
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
      throw std::runtime_error(
        std::string{"Function getaddrinfo() returned error: "}
        + gai_strerror(rv));
    }
  }

  // loop through serverinfo list.
  struct addrinfo* p;

  for (p = servinfo; p != nullptr; p = p->ai_next) {
    if ((sockfd_ = socket(p->ai_family, p->ai_socktype,
                          p->ai_protocol)) == -1) {
      throw std::runtime_error(
        std::string{"Function socket() returned error: "} + strerror(errno)
      );
    }

    if (::connect(sockfd_, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd_);
      continue;
    }

    break; // successfully connected
  }

  if (p == nullptr) {
    throw std::runtime_error(
      std::string{"Function connect() returned error: "} + strerror(errno)
    );
  }

  freeaddrinfo(servinfo);
  is_connected_ = true;
  return;
}

/**
 * Safely disconnect from the Nav2 machine.
 */
void Nav2Robot::disconnect() {
  stop();
  close(sockfd_);
  is_connected_ = false;
}

/**
 * Send msg across the TCP/IP connection to the robot. The command should be
 * part of the Nav2 commander interface specification as implemented on the
 * target robot.
 * @param  msg The message to be sent.
 * @return     Forwards the return value from write().
 */
int Nav2Robot::sendstr(std::string msg) {
  return send(sockfd_, msg.c_str(), msg.length(), 0);
}

/**
 * Takes a command to be sent to the Nav2 machine, adds a new line to the end
 * if it isn't already there and sends it. This function doesn't do a lot of
 * checking, so if there is a preexisting \n, it has to be the last character
 * in msg, otherwise, it isn't detected.
 * @param  msg The message to be sent.
 */
int Nav2Robot::sendline(std::string cmd) {
  if (cmd.back() == '\n')
    return sendstr(cmd);
  else
    return sendstr(cmd+'\n');
}

/**
 * This function has been superseded by the int(std::string) overload of the
 * same function. This is for C-style strings. The function is rather
 * pessimistic about the possibility of multiple newlines and multiple \0 in
 * the character array. If you have a \n that isn't at the end of the string,
 * everything after it is ignored. So if you pass multiple commands in the
 * string, only the first is sent and the rest are ignored.
 *
 * DEPRECATED: I'm going to remove this as soon as I can be sure it doesn't
 * hold any advantages over the int(std::string) overload. I've already taken
 * it out of all class methods herein, and I am removing it from all client
 * code under my control.
 * @param  cmd  The command line to send to the Nav2 server.
 * @param  size The number of characters in cmd.
 * @return      Returns the result from write().
 */
[[ deprecated ]]
int Nav2Robot::sendline(const char* cmd, const int size) {
  const char* s = cmd;
  const char* end = cmd + size;

  for (; (*s != '\0') && (s <= end); s++) {
    // If cmd already has a newline character read it into a well sized piece of
    // memory and ship it out.
    if (*s == '\n') {
      char* msg = (char*) calloc(sizeof(char),
                                 (s - cmd + 1) * sizeof(char));
      strncpy(msg, cmd, (s - cmd) / sizeof(char));
      int a = write(sockfd_, msg, strlen(msg));

      if (a < 0)
        throw std::runtime_error("Could not write to Nav2 device");

      free(msg);
      return a;
    }
  }

  // If we're here, then we've found a \0 and no \n, so create a new array and
  // add a \n and ship it out.
  char* msg = (char*) calloc(sizeof(char),
                             (s - cmd + 1) * sizeof(char));
  strncpy(msg, cmd, (s - cmd) / sizeof(char));
  strncat(msg, "\n", 1);
  int a = write(sockfd_, msg, strlen(msg));

  if (a < 0)
    throw std::runtime_error("Could not write to Nav2 device");

  free(msg);
  return a;
}

void Nav2Robot::set_host(std::string host) {
  if (is_connected_) throw
    std::logic_error("Cannot change Nav2Robot hostname while connected");
  else
    hostname_= host;
}

void Nav2Robot::set_port(int portno) {
  if (is_connected_) throw
    std::logic_error("Cannot change Nav2Robot port while connected");
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
std::string Nav2Robot::send_recv(std::string msg, int buffer_size) {
  // const int buffer_size = ;

  if (sendstr(msg) < 0)
    throw std::runtime_error(
      "Failed to write to Nav2 machine during send_recv. msg: " + msg
    );

  std::vector<char> buffer(buffer_size);
  std::string recieved_message;
  int bytes_received{0};

  do {
    bytes_received = recv(sockfd_, buffer.data(), buffer.size(), 0);

    if (bytes_received < 0)
      throw std::runtime_error(
        "Failed to read from Nav2 machine during pose update. msg: " + msg
      );
    else
      recieved_message.append(buffer.cbegin(), buffer.cend());
  } while (bytes_received == buffer_size);

  return recieved_message;

}

/**
 * Send a request to the turtle server for the robot's position, and pack it
 * into the first element of the state array, seeding the NMPC calculation
 * with up-to-date data.
 */
// void Nav2Robot::update_pose() {
//   // As much as I hate inconsistency, I can't justify using std::string for the
//   // read buffer
//   const int buff_size = 48;
//   char buff[buff_size] {};
//   if(sendstr("q\n") < 0)
//     throw std::runtime_error(
//       "Failed to write to Nav2 machine during pose update"
//     );
//   if(read(sockfd_, buff, buff_size) < 0)
//     throw std::runtime_error(
//       "Failed to read from Nav2 machine during pose update"
//     );
//   sscanf(buff, "%f %f %f", &pose_[0], &pose_[1], &pose_[2]);
//   //TODO return a tuple of the coordinates.
// }

std::tuple<float, float, float> Nav2Robot::update_pose() {
  std::string q_response = send_recv("q\n");
  sscanf(q_response.c_str(), "%f %f %f", &pose_[0], &pose_[1], &pose_[2]);
  return std::tuple<float, float, float> {pose_[0], pose_[1], pose_[2]};
}

// ## Wrapped Nav2 Commands
// =============================================================================

/**
 * Send Nav2 AV command to set absolute velocity in Cartesian coordinates.
 *
 * From Nav2 Interface specification:
 *
 * AV <vx> <vy> - Set the absolute velocity to the given vector, in meters per
 * second.
 *
 * @param  vx Speed in x-direction
 * @param  vy Speed in y-direction
 * @return    Forwards the return value from write() via sendstr().
 */
int Nav2Robot::av(float vx, float vy) {
  std::string msg = "AV "
                    + std::to_string(vx) + ' ' + std::to_string(vy) + '\n';
  return sendstr(msg);
}

/**
 * Send the Nav2 'BK' command.
 *
 * The four basic turtle commands: RT (right turn), LT (left turn),
 * FD (move forward), and BK (move backward).
 *
 * From the Nav2 interface documentation:
 *  BK <x>      - Move backward <x> meters
 *
 * @param  x Desired relative position of the robot in meters.
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::bk(float x) {
  return sendstr(std::string{"BK " + std::to_string(x) + '\n'});
}

/**
 * Send the Nav2 'FD' command.
 *
 * The four basic turtle commands: RT (right turn), LT (left turn),
 * FD (move forward), and BK (move backward).
 *
 * From the Nav2 interface documentation:
 *  FD <x>      - Move forward <x> meters
 *
 * @param  x Desired relative position of the robot in meters.
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::fd(float x) {
  return sendstr(std::string{"FD " + std::to_string(x) + '\n'});
}

/**
 * Send the Nav2 'LT' command.
 *
 * The four basic turtle commands: RT (right turn), LT (left turn),
 * FD (move forward), and BK (move backward).
 *
 * From the Nav2 interface documentation:
 *  LT <n>      - Turn left <n> degrees
 *
 * @param  n Desired relative orientation of the robot in degrees.
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::lt(float n) {
  return sendstr(std::string{"LT " + std::to_string(n) + '\n'});
}

/**
 * Send the Nav2 "MV" command to send the machine the specified absolute
 * position.
 *
 * From the Nav2 interface documentation:
 *
 *  TODO(T.T.) - Ask J.P. I think these are backwards.
 *  MV <n> <x>  - Move <x> meters in direction <n> degrees left of current
 *                heading
 *
 * @param  x [description]
 * @param  n [description]
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::mv(float x, float n) {
  std::string msg = "MV "
                    + std::to_string(x) + ' ' + std::to_string(n) + '\n';
  return sendstr(msg);
}

/**
 * Send Nav2 O command to set the orientation of the robot.
 *
 * From Nav2 Interface specification:
 *
 * O <n> - Set the absolute target orientation to <n> degrees.
 *
 * @param  n Desired absolute orientation of the robot in degrees.
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::o(float n) {
  return sendstr(std::string{"O " + std::to_string(n) + '\n'});
}

/**
 * Send a formatted CLI command to the VirtualME. The command sets the
 * internal reckoning of the robot to the origin oriented along the +x-axis.

 * @return Forwards the return value from write() via sendstr().
 */
int Nav2Robot::originate() {
  return sendstr("p 0 0 0\n");
}

/**
 * Send Nav2 'P' command to set the machine's internal understanding of it's
 * pose.
 *
 * From Nav2 Interface specification:
 *
 * P <x> <y> <n> - Set the robot's position and heading. Estimates will
 * be based on this ground truth.
 *
 * @param  x x-coordinate
 * @param  y y-coordinate
 * @param  n orientation angle in degrees.
 * @return   Forwards the return value from write() via sendstr()
 */
int Nav2Robot::p(float x, float y, float n) {
  std::string msg = "P " + std::to_string(x) + ' '
                    + std::to_string(y) + ' '
                    + std::to_string(n) + '\n';
  return sendstr(msg);
}

/**
 * Send Nav2 'Q' command to poll the machine's internal rekoning of pose.
 *
 * From Nav2 Interface specification:
 *
 * Q - Estimate the robot's position and heading. This command also outputs
 * the number of turtle commands remaining in the queue.
 *
 * NB: Also see Nav2Robot::update_pose().
 *
 * @return [description]
 */
std::tuple<float, float, float, int> Nav2Robot::q() {
  std::string q_response = send_recv("q\n");
  float x, y, n;
  int q;
  sscanf(q_response.c_str(), "%f %f %f %d", &x, &y, &n, &q);
  return std::tuple<float, float, float, int> {x, y, n, q};
}

/**
 * Send the Nav2 'RT' command.
 *
 * The four basic turtle commands: RT (right turn), LT (left turn),
 * FD (move forward), and BK (move backward).
 *
 * From the Nav2 interface documentation:
 *  RT <n>      - Turn right <n> degrees
 *
 * @param  n Desired relative orientation of the robot in degrees.
 * @return   Forwads the return value from write() via sendstr()
 */
int Nav2Robot::rt(float n) {
  return sendstr(std::string{"RT " + std::to_string(n) + '\n'});
}

/**
 * Stop the robot by sending the Nav2 'S' command.
 *
 * From the Nav2 interface documentation:
 *  S                - Stop the robot
 *
 * @return Forwards the return value from write().
 */
int Nav2Robot::stop() {
  return sendstr("\ns\n");
}

/**
 * Send a formatted CLI command to the VirtualME. The command sets the
 * velocity and turn-rate of the Nav2Robot (see Nav2 documentation for 'v <n>
 * <s> [<t>]'.
 */
int Nav2Robot::v(float theta, float v, float Dth) {
  std::string msg = "v " + std::to_string(theta) + ' ' + std::to_string(v)
                    + ' ' + std::to_string(Dth) + '\n';
  return sendstr(msg);
}

// ## Specialty Commands
// =============================================================================

/**
 * Sends Nav2 "AV" command, but the parameter(s) is the polar representation
 * of the desired velocity vector and the conversion is done on the fly.
 * @param  v     Desired absolute speed.
 * @param  theta Desired absolute direction in degrees.
 * @return       Forwards the return value from write().
 */
int Nav2Robot::avv(float v, float theta) {
  float rads = degToRad(theta);
  std::string msg = "AV " + std::to_string(std::cos(rads) * v) + ' '
                    + std::to_string(std::sin(rads) * v) + '\n';
  return sendstr(msg);
}

// TODO(TT): A struct that takes a string (cmd) and a vector of arguments and
// formats them into a command; This replaces the functionality of the python
// version's command() member.
