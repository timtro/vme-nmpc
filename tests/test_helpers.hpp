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

#ifndef VME_NMPC_TESTS_TEST_HELPERS_HPP_
#define VME_NMPC_TESTS_TEST_HELPERS_HPP_

#include <iostream>
#include <valarray>

namespace thlp {
inline void printArray(std::valarray<float> a) {
  for (auto each : a) std::cout << each << ", ";

  std::cout << std::endl;
}

template <typename T, typename V>
bool eachInArrayIsApprox(T array, V expectedValue, V absoluteError) {
  return std::all_of(std::begin(array), std::end(array),
                     [expectedValue, absoluteError](auto val) {
                       return expectedValue - absoluteError <= std::abs(val) &&
                              std::abs(val) <= expectedValue + absoluteError;
                     });
}

bool eachIsTrue(const std::valarray<bool>& comparison) {
  bool equals = true;
  for (auto item : comparison) equals &= item;
  return equals;
}

template <typename T>
bool arraysAreAbsEqual(const T& a, const T& b,
                       decltype(a[0] + b[0]) absoluteError) {
  return eachIsTrue(std::abs(std::abs(a) - std::abs(b)) < absoluteError);
}
}


#endif // VME_NMPC_TESTS_TEST_HELPERS_HPP_
