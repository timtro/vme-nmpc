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

#ifndef VME_NMPC_SRC_TARGET_HPP_
#define VME_NMPC_SRC_TARGET_HPP_

#include "linear.hpp"
#include <memory>
#include <deque>

struct Target {
  fp_point2d locus;
  decltype(locus.x)& x;
  decltype(locus.x)& y;
  decltype(locus.x) tolerance;

  Target(float x, float y, float tol)
      : locus{x, y}, x{locus.x}, y{locus.y}, tolerance{tol} {}
};

class TargetContainer {
  std::deque<std::unique_ptr<Target>> targets;

 public:
  void push_back(std::unique_ptr<Target>);
  void emplace_back(Target*);
  void push_front(std::unique_ptr<Target>);
  void pop_back();
  void pop_front();
  void clear();
  size_t size();
  void clearContainer();
  [[deprecated]] bool hasTargets();
  bool empty() const noexcept;
  Target& operator[](const int i);
};

#endif  // VME_NMPC_SRC_TARGET_HPP_