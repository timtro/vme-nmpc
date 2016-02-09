/* This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 Timothy A.V. Teatro - All rights Reserved
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.

* * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VME_NMPC_SRC_NMPCKERNEL_HPP_
#define VME_NMPC_SRC_NMPCKERNEL_HPP_

#include "linear.hpp"
#include "Subject.hpp"
#include "VMeCommand.hpp"
#include "NmpcMinimizer.hpp"
#include "NmpcModel.hpp"
#include "PathPlanner.hpp"
#include "VMeLogger.hpp"
#include "AggregatorInitializer.hpp"

class NmpcMinimizer;

using vMeModelType = NmpcModel<SeedPackage, up_VMeCommand>;

/**
 * The kernel organizes the NMPC calculation. Its interface is meant to be used
 * by a high level supervisory routine. It aggregates the objects related to the
 * NMPC calculation: the model, minimizer, PathPlanner and logger.
 *
 * The kernel can be used to pass the seed to the model, step through the NMPC
 * calculation and it doles out commands for execution.
 */
class VMeNmpcKernel : public Subject {
  vMeModelType* model;
  NmpcMinimizer* minimizer;
  VMeLogger* logger;
  PathPlanner<SeedPackage>* planner;
  std::unique_ptr<VMeLogger> noOpLogger{nullptr};  // Default log if none given.

  fptype targetDistanceTolerance{0};
  unsigned cmdsExecutedFromCurrentHorizon{0};

 public:
  VMeNmpcKernel(const VMeNmpcKernel&) = delete;
  VMeNmpcKernel& operator=(const VMeNmpcKernel&) = delete;
  VMeNmpcKernel(AggregatorInitializer&);

  up_VMeCommand next_command();
  void seed(SeedPackage&);
  void nmpc_step(SeedPackage&);
  void run();
};

class MinimizerReachedIterationLimit : public std::exception {
  virtual const char* what() const noexcept override {
    return "Minimizer reached its specified iteration limit.";
  }
};

#endif  // VME_NMPC_SRC_NMPCKERNEL_HPP_
