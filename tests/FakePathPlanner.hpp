/* This file is part of This program.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * This program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VME_NMPC_TESTS_FAKEPATHPLANNER_HPP_
#define VME_NMPC_TESTS_FAKEPATHPLANNER_HPP_

#include "../src/PathPlanner.hpp"
#include "../src/SeedPackage.hpp"
#include "../src/AggregatorInitializer.hpp"

class FakePathPlanner : public PathPlanner<SeedPackage> {
  SeedPackage seed;

 public:
  FakePathPlanner(AggregatorInitializer& init) : seed(init) {
    init.bindIntoAggregator(this);
  }
  ~FakePathPlanner() = default;
  SeedPackage& getSeed() { return this->seed; }
};

#endif  // VME_NMPC_TESTS_FAKEPATHPLANNER_HPP_