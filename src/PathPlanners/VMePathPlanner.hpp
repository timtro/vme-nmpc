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

#ifndef VME_NMPC_SRC_VMEPATHPLANNER_HPP_
#define VME_NMPC_SRC_VMEPATHPLANNER_HPP_

#include "../Target.hpp"
#include "../AggregatorInitializer.hpp"
#include "../SeedPackage.hpp"

class VMePathPlanner : public PathPlanner<SeedPackage> {
  SeedPackage seed;
  TargetContainer targets;
  point2d targetUnitVector{0.0, 0.0};
  float distanceToTarget{0.0};
  float timeInterval{0.0};

 public:
  VMePathPlanner(AggregatorInitializer&);
  ~VMePathPlanner() = default;

  void computeTargetMetrics();
  void computeTrackingErrors() noexcept;
  virtual SeedPackage& getSeed();

};

#endif // VME_NMPC_SRC_VMEPATHPLANNER_HPP_