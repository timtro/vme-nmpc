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

#include "VMePathPlanner.hpp"

VMePathPlanner::VMePathPlanner(AggregatorInitializer& init)
    : seed(init.get_nmpcHorizon()) {
  timeInterval = init.get_timeInterval();
  seed.vref = init.get_cruiseSpeed();
  targets = init.targets;
  init.bind_into_aggregator(this);
}

void VMePathPlanner::computeTargetMetrics() {
  targetUnitVector.x = (*targets)[0].x - seed.pose.x;
  targetUnitVector.y = (*targets)[0].y - seed.pose.y;
  distanceToTarget = std::sqrt(dot(targetUnitVector, targetUnitVector));
  targetUnitVector /= distanceToTarget;
}

void VMePathPlanner::compute_tracking_errors() noexcept {
  for (unsigned k = 0; k < seed.xref.size(); ++k) {
    seed.xref[k] = seed.pose.x +
                   seed.vref[k] * targetUnitVector.x * (k + 1) * timeInterval;
    seed.yref[k] = seed.pose.y +
                   seed.vref[k] * targetUnitVector.y * (k + 1) * timeInterval;
  }
}

SeedPackage& VMePathPlanner::get_seed() {
  seed.pose = stateEstimateRetriever().pose;
  computeTargetMetrics();
  compute_tracking_errors();
  return this->seed;
}

bool VMePathPlanner::is_continuing() {
  {
    if (distanceToTarget < (*targets)[0].tolerance) {
      targets->pop_front();
      if (targets->empty())
        return false;
      else
        return true;
    } else
      return true;
  }
}

void VMePathPlanner::set_stateEstimateRetriever(
    std::function<SeedPackage()> fun) {
  stateEstimateRetriever = fun;
}
