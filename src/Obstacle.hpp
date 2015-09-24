/*
 * vme-nmpc/src/Obstacle.hpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-24
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

#ifndef VME_NMPC_SRC_OBSTACLE_H__
#define VME_NMPC_SRC_OBSTACLE_H__

#include "linear.hpp"

#include <utility> // for std::pair

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

  Point2R position;

  virtual fptype dist(Point2R) = 0;
  virtual fptype phi(Point2R) = 0;
  virtual Point2R gradPhi(Point2R) = 0;

};

class ObstacleContainer {
  std::vector<Obstacle*> obstacles;

 public:
  Point2R gradPhi(Point2R);

};


#endif // VME_NMPC_SRC_OBSTACLE_H__