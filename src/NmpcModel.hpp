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

#ifndef __VME_NMPC_SRC_NMPCMODEL_HPP__
#define __VME_NMPC_SRC_NMPCMODEL_HPP__

#include "Obstacle.hpp"
#include "linear.hpp"
#include "NmpcInitPkg.hpp"

template <typename seedType, typename tgtType, typename cmdType>
class NmpcModel {
 public:
  virtual ~NmpcModel() = default;
  virtual void seed(seedType, tgtType) = 0;
  virtual void seed(seedType) = 0;
  virtual void forecast() = 0;
  virtual void setTrackingErrors() = 0;
  virtual void computePathPotentialGradient(ObstacleStack &obstacles) = 0;
  virtual void computeGradient() = 0;
  virtual fptype getTargetDistance() = 0;
  virtual cmdType getCommand(int) = 0;
  virtual unsigned getHorizonSize() const = 0;
};

#endif  // __VME_NMPC_SRC_NMPCMODEL_HPP__