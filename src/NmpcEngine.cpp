/*
 * vme-nmpc/src/NmpcEngine.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-25
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 - Timothy A.V. Teatro
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

// TODO(TT): Test speed of storing sintk vs letting the compiler optimize it.

// NB(TT to TT): to convert from old notation:
// s/qu\[([^\]]*)\]\.([a-zA-Z0-9]*)/\2\[\1\]/g

#include "NmpcEngine.hpp"
#include "Obstacle.hpp"

#include <vector>
#include <valarray>

/*
Make state space styff accessible through agetters. Lagrange portion doesent
need utesting and can be fkushed out through gradient.
 */

NmpcEngine::NmpcEngine(NmpcInitPkg& ini) :
  N{ini.N}, m{ini.m}, n{ini.n}, T{ini.T}, dg{ini.dg},
  cruising_speed{ini.cruising_speed}, Q{ini.Q}, Q0{ini.Q0}, R{ini.R} {
  x = fpArray(0.f, N);
  Dx = fpArray(0.f, N);
  y = fpArray(0.f, N);
  Dy = fpArray(0.f, N);
  th = fpArray(0.f, N);
  Dth = fpArray(0.f, N);
  v = fpArray(0.f, N);
  ex = fpArray(0.f, N);
  ey = fpArray(0.f, N);
  px = fpArray(0.f, N);
  pDx = fpArray(0.f, N);
  py = fpArray(0.f, N);
  pDy = fpArray(0.f, N);
  pth = fpArray(0.f, N);
  grad = fpArray(0.f, N);
  last_grad = fpArray(0.f, N);
  targets = std::deque<Target>();
}