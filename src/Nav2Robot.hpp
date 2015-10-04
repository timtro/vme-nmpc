/* Copyright (C) 2015 by Timothy A.V. Teatro

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VME_NMPC_SRC_NAV2ROBOT_HPP__
#define __VME_NMPC_SRC_NAV2ROBOT_HPP__

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

#endif // __VME_NMPC_SRC_NAV2ROBOT_HPP__