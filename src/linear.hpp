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

#ifndef VME_NMPC_SRC_LINEAR_HPP_
#define VME_NMPC_SRC_LINEAR_HPP_

#include "typedefs.h"
#include "CrossWing/linear.h"

typedef Point<fptype, 2> fp_point2d;

template <typename R>
class XYTh {
 public:
  static const unsigned size = 4;
  R x, y, th;
  XYTh() {}
  explicit XYTh(R val) : x(val), y(val), th(val){};
  template <typename R2>
  explicit XYTh(const Point<R2, 4>& p) {
    for (unsigned i = 0; i < size; ++i) (*this)[i] = R(p[i]);
  }
  XYTh(R x, R y, R th)
      : x(std::move(x)), y(std::move(y)), th(std::move(th)){};
  typedef R value_type;
  template <typename R2>
  XYTh<R>& operator=(const Point<R2, 3>& b) {
    for (unsigned i = 0; i < size; ++i) (*this)[i] = R(b[i]);
    return *this;
  }
};

// Dot product/
template <typename R, int n>
inline auto dot(const Point<R, n> a, const Point<R, n>& b) -> R {
  R tot{0};

  for (unsigned i = 0; i < a.size; ++i) tot += a[i] * b[i];

  return tot;
}

template <typename R>
inline auto dot(const Point<R, 2>& a, const Point<R, 2>& b) -> R {
  return a.x * b.x + a.y * b.y;
}

using xyth = XYTh<fptype>;

// (TT) Added std:: to trig functions so the type of ANGLE is mirrored in the
// (returned Point<T, 2>..
template <typename ANGLE>
inline auto unitVector(ANGLE theta) -> Point<decltype(std::cos(theta)), 2> {
  return Point<decltype(std::cos(theta)), 2>(std::cos(theta), std::sin(theta));
}

#endif  // VME_NMPC_SRC_LINEAR_HPP_
