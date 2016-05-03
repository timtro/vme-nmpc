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

/**
 * A structure embodying the notion of a target. The target has a locus (the x-
 * and y-coordinates) as well as a tolerance. The tolerance creates a ball
 * around the locus into which if the plant enters, the target may be marked as
 * reached.
 */
struct Target {
  fp_point2d locus;
  decltype(locus.x)& x;
  decltype(locus.x)& y;
  decltype(locus.x) tolerance;

  Target(float x, float y, float tol)
      : locus{x, y}, x{locus.x}, y{locus.y}, tolerance{tol} {}
};

/**
 * A container for targets which abstracts away implementation and provides a
 * convenient interface.
 */
class TargetContainer {
  using InternalContainer = std::deque<std::unique_ptr<Target>>;
  InternalContainer targets;
  std::unique_ptr<Target> previousTarget{new Target(0, 0, 0)};

 public:
  void push_back(std::unique_ptr<Target>);
  void emplace_back(Target*);
  void push_front(std::unique_ptr<Target>);
  void pop_back();
  void pop_front();
  void clear();
  void create_front(fp_point2d, fptype);
  size_t size();
  [[deprecated]] bool has_targets();
  bool empty() const noexcept;
  Target& operator[](const int i);

  using const_iterator = InternalContainer::const_iterator;

  const_iterator begin() const { return targets.begin(); }
  const_iterator end() const { return targets.end(); }
  const_iterator cbegin() const { return targets.cbegin(); }
  const_iterator cend() const { return targets.cend(); }

};

#endif  // VME_NMPC_SRC_TARGET_HPP_
