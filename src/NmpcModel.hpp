/*
 * vme-nmpc/src/NmpcModel.hpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-09-16
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

#ifndef VME_NMPC_SRC_NMPCMODEL_HPP__
#define VME_NMPC_SRC_NMPCMODEL_HPP__

#include "Obstacle.hpp"
#include "linear.hpp"
#include <deque>
#include <vector>
#include <valarray>

struct NmpcInitPkg {
  int N;
  int m;
  int n;
  float T;
  float dg;
  float cruising_speed;
  float Q;
  float Q0;
  float R;
};

class NmpcModel {

 public:

  int N;
  int m;
  int n;
  float T;
  float dg;
  float cruising_speed;
  float Q;
  float Q0;
  float R;

  //! The x-coordinate.
  std::valarray<float> x;
  //! The time rate-of-change of x.
  std::valarray<float> Dx;
  //! The y-coordinate.
  std::valarray<float> y;
  //! The time rate-of-change of y.
  std::valarray<float> Dy;
  //! The angle from the x-axis of the direction of travel.
  std::valarray<float> th;
  //! The steering rate. That is, the time rate-of-change of th.
  std::valarray<float> Dth;
  //! The radial component of speed.
  std::valarray<float> v;
  //! The error of the x-coordinate.
  std::valarray<float> ex;
  //! The error of the y-coordinate.
  std::valarray<float> ey;
  // The Lagrange multipliers

  std::valarray<float> px;
  std::valarray<float> pDx;
  std::valarray<float> py;
  std::valarray<float> pDy;
  std::valarray<float> pth;

  std::valarray<float> grad;
  std::valarray<float> last_grad;

  NmpcModel(NmpcInitPkg&);

  void seed();
  void seed(XYVTh<float>);
  void forecast();
};

#endif // VME_NMPC_SRC_NMPCMODEL_HPP__