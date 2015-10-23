/*
 * vme-nmpc/src/obstacle-classes/PointObstacle.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-24
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

//TODO(TT): wrap linear.h to insert my own typedef for the sake of brevity and
//generics.

#include "PointObstacle.hpp"

#include "../linear.hpp"

#include <cmath>
#include <initializer_list>

#include <cstdio>

PointObstacle::PointObstacle(fp_point2d position, fptype pwr, fptype eps) :
  position{position}, pwr{pwr}, eps{eps} {}

/**
 * Standard Euclidian distance metric.
 * @param  rx reference x-coordinate.
 * @param  ry reference y-coordinate.
 * @return    The distance from the centre of the point obstacle to the
 * reference point.
 */
auto PointObstacle::dist(fp_point2d xr) -> decltype(xr.x) {
  auto disp = xr - position;
  return std::sqrt(dot(disp, disp));
}

/**
 * The value of the scalar potential field
 * @param  rx reference x-coordinate.
 * @param  ry reference y-coordinate.
 * @return    The scalar field value.
 */
auto PointObstacle::phi(fp_point2d refPoint) -> decltype(refPoint.x) {
  return 1/(  std::pow( dist(refPoint), pwr ) + eps  );
}

/**
 * Computes the gradient of potential WRT vector r as:
 *
 *                pwr - 1
 *  dPhi     pwr r
 *  ---- = - ------------- ,
 *   dr        pwr       2
 *           (r    + eps)
 *
 * or more explicitly in terms of x and y,
 *                       x
 *   [- ------------------------------------ ,
 *            2    2         2    2        2
 *      sqrt(y  + x ) (sqrt(y  + x ) + eps)
 *                                                     y
 *                                  - ------------------------------------]
 *                                          2    2         2    2        2
 *                                    sqrt(y  + x ) (sqrt(y  + x ) + eps)
 *
 * @param  rx reference x-coordinate.
 * @param  ry reference y-coordinate.
 * @return    A pair, the x- and y-components of the gradient vector.
 */
decltype(PointObstacle::position) PointObstacle::gradPhi(fp_point2d refPoint) {
  auto disp = position - refPoint; // = - (refPoint - position)
  auto d = disp.x*disp.x + disp.y*disp.y;
  auto num = pwr*disp*std::pow(d, pwr/2 - 1);
  auto den = std::pow( std::pow(d, pwr/2) + eps , 2);
  return num/den;
}
