/*
 * vme-nmpc/src/ClArgs.hpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-22
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

#ifndef VME_NMPC_SRC_CLOPTS_HPP__
#define VME_NMPC_SRC_CLOPTS_HPP__

#include <string>

struct ClArgs {

  std::string infile;
  std::string host;
  int port;
  bool verbose;
  bool quiet;

  ClArgs(int, char**);
};

#endif // VME_NMPC_SRC_CLOPTS_HPP__