/* This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
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

#ifndef VME_NMPC_VIRTUALMECOMMAND_HPP
#define VME_NMPC_VIRTUALMECOMMAND_HPP

#include "Nav2Robot.hpp"
#include <memory>

struct VirtualMeCommand {
  virtual int execute(Nav2Robot &) = 0;
};

struct VMeStop : public VirtualMeCommand {
  virtual int execute(Nav2Robot &rob);
};

struct VMeV : public VirtualMeCommand {
  float v = 0;
  float th = 0;
  float Dth = 0;

  VMeV(float th, float v, float Dth) : v{v}, th{th}, Dth{Dth} {}

  virtual int execute(Nav2Robot &rob);
};

struct VMeNullCmd : public VirtualMeCommand {
  virtual int execute(Nav2Robot &rob);
};

using up_VirtualMeCommand = std::unique_ptr<VirtualMeCommand>;

#endif  // VME_NMPC_VIRTUALMECOMMAND_HPP