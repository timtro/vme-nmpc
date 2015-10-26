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

#ifndef VME_NMPC_SRC_NMPCMODEL_HPP_
#define VME_NMPC_SRC_NMPCMODEL_HPP_

#include "Obstacle.hpp"
#include "linear.hpp"
#include "NmpcInitPkg.hpp"

template <typename seedType, typename tgtType, typename cmdType>
class NmpcModel {
 public:
  virtual ~NmpcModel() = default;
  virtual unsigned getHorizonSize() const = 0;
  virtual fptype getTargetDistance() const noexcept = 0;
  virtual void seed(seedType, tgtType) = 0;
  virtual void seed(seedType) = 0;
  virtual void computeForecast() noexcept = 0;
  virtual void computeTrackingErrors() noexcept = 0;
  virtual void computePathPotentialGradient(
      ObstacleContainer &obstacles) noexcept = 0;
  virtual void computeGradient() noexcept = 0;
  virtual cmdType getCommand(int) const = 0;
};

#endif  // VME_NMPC_SRC_NMPCMODEL_HPP_