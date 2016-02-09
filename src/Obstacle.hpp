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

#ifndef VME_NMPC_SRC_OBSTACLE_HPP_
#define VME_NMPC_SRC_OBSTACLE_HPP_

#include "linear.hpp"

#include <memory>
#include <mutex>

/**
 * An abstract base class defining the interface that will be required for the
 * NMPC computations. The NMPC calculation needs to know the potential and the
 * potential gradient as a function of position from any obstacle.
 *
 * In order to compute those quantities, the class will almost certainly need
 * to compute some notion of distance from the obstacle to any given point. I
 * make the access to the distance metric function public in case it because
 * useful for future add-ons, such as a global path planner.
 */
struct Obstacle {
  virtual fptype dist(fp_point2d) = 0;
  virtual fptype phi(fp_point2d) = 0;
  virtual fp_point2d gradient_phi(fp_point2d) = 0;
};

/**
 * A container for obstacles to separate implementation from usage. For some
 * cases, a simple linked list will do, but in complex environments where
 * obstacles must be updated, added and remove, this will need a hash-table in
 * the back end.
 */
class ObstacleContainer {
  std::vector<std::unique_ptr<Obstacle>> obstacles;
  mutable std::mutex mutex;

 public:
  fp_point2d gradient_phi(fp_point2d);
  void push(Obstacle *obs);
  void push_unique_ptr(std::unique_ptr<Obstacle>);
  void pop();
  size_t size();
  void clear();
  [[deprecated]] bool has_obstacles() const noexcept;
  bool empty() const noexcept;
  std::unique_ptr<Obstacle> &operator[](const int i);
};

#endif  // VME_NMPC_SRC_OBSTACLE_HPP_
