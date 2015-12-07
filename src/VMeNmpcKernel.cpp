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

#include "VMeNmpcKernel.hpp"
#include "NmpcModel.hpp"

using vMeModelType = NmpcModel<SeedPackage, up_VMeCommand>;

VMeNmpcKernel::VMeNmpcKernel(AggregatorInitializer& init)
    : targetDistanceTolerance(init.get_targetDistanceTolerance()) {
  // TODO Safety checks
  init.aggregatorCompletionSafetyCheck();
  model = init.model;
  minimizer = init.minimizer;
  planner = init.planner;

  if (init.logger == nullptr) {
    noOpLogger = std::make_unique<VMeLogger>();
    logger = noOpLogger.get();
  } else
    logger = init.logger;
}

up_VMeCommand VMeNmpcKernel::nextCommand() {
  if (cmdsExecutedFromCurrentHorizon++ >= model->get_horizonSize())
    return up_VMeCommand{new VMeNullCmd()};
  else
    return model->retrieveCommand(cmdsExecutedFromCurrentHorizon);
}

void VMeNmpcKernel::seed(SeedPackage& seed) {
  model->seed(seed);
  cmdsExecutedFromCurrentHorizon = 0;
}

void VMeNmpcKernel::nmpcStep(SeedPackage& seed) {
  cmdsExecutedFromCurrentHorizon = 0;
  model->seed(seed);
  auto minimizerStatus = minimizer->solveOptimalControlHorizon();
  if (minimizerStatus == MinimizerCode::reachedIterationLimit)
    throw MinimizerReachedIterationLimit();
  logger->logModelState();
  logger->logMinimizerState();
  notify();
}

void VMeNmpcKernel::run() {
  do {
    nmpcStep(planner->getSeed());
  } while (planner->isContinuing());
}

vMeModelType* VMeNmpcKernel::_getModelPointer_() { return model; }

NmpcMinimizer* VMeNmpcKernel::_getMinimizerPointer_() { return minimizer; }