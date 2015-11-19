/*
 * vme-nmpc/src/obstacle-classes/PointObstacle.hpp
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

#ifndef VME_NMPC_SRC_OBSTACLE_CLASSES_POINTOBSTACLE_HPP__
#define VME_NMPC_SRC_OBSTACLE_CLASSES_POINTOBSTACLE_HPP__

#include "../Obstacle.hpp"

#include <initializer_list>

/**
 * A definition for a simple point obstacle with potiential field
 *
 *               1
 *  Phi(r) = ----------
 *            pwr
 *           r   + eps
 * where r is the shortest distance to the obstacle.

 * The obstacle may be a single point, or my be a list of nodes which are
 * assumed to be connected by walls. If a point is repeated in the list,
 * then the obstacle will have some sort of closed geometry.
 */

struct PointObstacle : public Obstacle {

  fp_point2d position;
  fptype pwr, eps;

  PointObstacle(fp_point2d, fptype, fptype);

  virtual fptype dist(fp_point2d);
  virtual fptype phi(fp_point2d);
  virtual fp_point2d gradPhi(fp_point2d);


};

#endif // VME_NMPC_SRC_OBSTACLE_CLASSES_POINTOBSTACLE_HPP__