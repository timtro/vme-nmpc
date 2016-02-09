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

#include "../AggregatorInitializer.hpp"
#include "../SeedPackage.hpp"
#include "../Target.hpp"

class FailedToProvideFunctionToRetrieveStateEstimate : public std::exception {
  const char* what() const noexcept override {
    return "Use the VMePathPlanner::set_stateEstimateRetriever method to "
           "provide a std::function to retrieve the state estimate for seed "
           "construction.";
  }
};

class VMePathPlanner : public PathPlanner<SeedPackage> {
  SeedPackage seed;
  TargetContainer* targets;
  point2d targetUnitVector{0.0, 0.0};
  float distanceToTarget{0.0};
  float timeInterval{0.0};
  std::function<SeedPackage()> stateEstimateRetriever = []() {
    throw FailedToProvideFunctionToRetrieveStateEstimate();
    SeedPackage defaultSeed;
    return defaultSeed;
  };

 public:
  VMePathPlanner(AggregatorInitializer&);
  ~VMePathPlanner() = default;

  virtual SeedPackage& get_seed();
  virtual bool is_continuing();

  void computeTargetMetrics();
  void compute_tracking_errors() noexcept;

  void set_stateEstimateRetriever(std::function<SeedPackage()>);
};

#endif  // VME_NMPC_SRC_VMEPATHPLANNER_HPP_
