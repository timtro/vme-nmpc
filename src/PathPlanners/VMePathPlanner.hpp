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
  VMePathPlanner(AggregatorInitializer& init) : seed(init) {
    timeInterval = init.get_timeInterval();
    seed.vref = init.get_cruiseSpeed();
  }

  void computeTargetMetrics() {
    targetUnitVector.x = targets[0].x - seed.pose.x;
    targetUnitVector.y = targets[0].y - seed.pose.y;
    distanceToTarget = std::sqrt(dot(targetUnitVector, targetUnitVector));
    targetUnitVector /= distanceToTarget;
  }

  void computeTrackingErrors() noexcept {
    point2d targetUnitVector;
    for (unsigned k = 1; k < seed.xref.size(); ++k) {
      seed.xref[k] =
          seed.pose.x + seed.vref[k] * targetUnitVector.x * k * timeInterval;
      seed.yref[k] =
          seed.pose.y + seed.vref[k] * targetUnitVector.y * k * timeInterval;
    }
  }

};

#endif // VME_NMPC_SRC_VMEPATHPLANNER_HPP_