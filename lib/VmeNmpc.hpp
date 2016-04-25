/*
 * vme-nmpc/lib/VmeNmpc.hpp
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

#ifndef VME_NMPC_LIB_VMENMPC_HPP_
#define VME_NMPC_LIB_VMENMPC_HPP_

#include <string>

class VmeNmpc {
  std::string hostname_;
  unsigned int portno_;
  int sockfd;
  bool is_connected_{false};

 public:
  VmeNmpc& operator=(const VmeNmpc&) = delete;
  VmeNmpc(const VmeNmpc&) = delete;
  VmeNmpc();
  VmeNmpc(std::string, unsigned int);
  ~VmeNmpc();

  void connect();
  void disconnect();
  int sendstr(std::string);
  int sendline(std::string);
  std::string send_recv(std::string, int = 128);
  void set_host(std::string);
  void set_port(int);

  // Wrapped CLI commands:
  int at(float, float, float);
  int clear_targets();

};

#endif  // VME_NMPC_LIB_VMENMPC_HPP_
