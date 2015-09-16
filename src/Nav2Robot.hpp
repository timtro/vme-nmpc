/*
 * vme-nmpc/src/Nav2Robot.hpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-17
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 by Timothy A.V. Teatro
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

#ifndef VME_NMPC_SRC_ROBOT_H__
#define VME_NMPC_SRC_ROBOT_H__

#include <string>
#include <tuple>

class Nav2Robot {

  std::string hostname_;
  unsigned int portno_;
  int sockfd_;
  float pose_[3];
  bool is_connected_{false};

  Nav2Robot& operator=(const Nav2Robot&) = delete;
  Nav2Robot(const Nav2Robot&) = delete;

 public:

  Nav2Robot();
  Nav2Robot(std::string, unsigned int);
  ~Nav2Robot();

  void connect();
  void disconnect();
  int sendstr(std::string);
  int sendline(const char* , int);
  int sendline(std::string);
  std::string send_recv(std::string, int = 128);
  void set_host(std::string);
  void set_port(int);
  std::tuple<float, float, float> update_pose();
  // Wrapped Nav2 commands:
  int av(float, float);
  int bk(float);
  int fd(float);
  int lt(float);
  int mv(float, float);
  int o(float);
  int originate();
  int p(float, float, float);
  std::tuple<float, float, float, int> q();
  int rt(float);
  int r(float);
  int stop();
  int v(float, float, float);
  // Specialty commands:
  int avv(float, float);
};

#endif // VME_NMPC_SRC_ROBOT_H__