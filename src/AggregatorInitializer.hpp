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

#ifndef VME_NMPC_NMPCINITPKG_HPP_
#define VME_NMPC_NMPCINITPKG_HPP_

#include "typedefs.h"
#include "linear.hpp"
#include "NmpcModel.hpp"
#include "NmpcMinimizer.hpp"
#include "VMeLogger.hpp"
#include "VMeCommand.hpp"
#include <memory>

class ObstacleContainer;

struct AggregatorInitializer {
  unsigned int horizonSize{0};
  fptype timeInterval{0};
  fptype cruiseSpeed{0};
  fptype Q{0};
  fptype Q0{0};
  fptype R{0};
  fptype sdStepFactor{0};
  fptype sdConvergenceTolerance{0};
  NmpcModel<xyth, fp_point2d, up_VMeCommand>* model{nullptr};
  NmpcMinimizer* minimizer{nullptr};
  VMeLogger* logger{nullptr};
  ObstacleContainer* obstacles{nullptr};

  bool modelBindingSafetyCheck();
  bool minimizerBindingSafetyCheck();
  bool loggerBindingSafetyCheck();
  bool aggregatorCompletionSafetyCheck();
  void bindIntoAggregator(NmpcModel<xyth, fp_point2d, up_VMeCommand>*);
  void bindIntoAggregator(NmpcMinimizer*);
  void bindIntoAggregator(VMeLogger*);
};

class ModelMustBeInitializedBeforeMinimizerOrLogger : public std::exception {};
class InitPkgCanOnlyBeUsedOnceToInitializeAModel : public std::exception {};
class InitPkgDoesNotContainPointerToAModel : public std::exception {};
class InitPkgDoesNotContainPointerToAMinimizer : public std::exception {};
class InitPkgAlreadyHasBoundMinimizer : public std::exception {};

#endif  // VME_NMPC_NMPCINITPKG_HPP_
