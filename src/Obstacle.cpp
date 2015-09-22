/*
 * Obstacle.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-31
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

#include "linear.hpp"
#include "Obstacle.hpp"

Point2R ObstacleContainer::gradPhi(Point2R refPoint) {

  auto sum = Point2R {0.f, 0.f};

  for (auto each : obstacles)
    sum += each->gradPhi(refPoint);

  return sum;
}