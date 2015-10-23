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

#ifndef VME_NMPC_SRC_TRIG_HPP_
#define VME_NMPC_SRC_TRIG_HPP_

template <typename T>
constexpr T pi{T(3.141592653589793238462643383279502884197169399375105821)};

template <typename T>
constexpr T pi_180{T(pi<T> / 180)};

template <typename T>
T degToRad(T degs) {
  return degs * pi_180<T>;
}

template <typename T>
T radToDeg(T rads) {
  return rads / pi_180<T>;
}

#endif  // VME_NMPC_SRC_TRIG_HPP_
