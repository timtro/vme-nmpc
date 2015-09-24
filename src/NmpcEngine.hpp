/*
 * vme-nmpc/src/NmpcEngine.hpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-25
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

#ifndef VME_NMPC_SRC_NMPCENGINE_HPP__
#define VME_NMPC_SRC_NMPCENGINE_HPP__

#include "Obstacle.hpp"
#include <deque>
#include <vector>
#include <valarray>


struct NmpcInitPkg {
  int N;
  int m;
  int n;
  fptype T;
  fptype dg;
  fptype cruising_speed;
  fptype Q;
  fptype Q0;
  fptype R;
};



class NmpcEngine {


};

#endif // VME_NMPC_SRC_NMPCENGINE_HPP__