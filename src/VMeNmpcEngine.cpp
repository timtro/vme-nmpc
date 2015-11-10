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

#include "VMeNmpcEngine.hpp"
#include "NmpcModel.hpp"

VMeNmpcEngine::VMeNmpcEngine(AggregatorInitializer& init) {
  // TODO Safety checks
  init.aggregatorCompletionSafetyCheck();
  model = init.model;
  minimizer = init.minimizer;

  if (init.logger == nullptr) {
    noOpLogger = std::make_unique<VMeLogger>();
    logger = noOpLogger.get();
  } else
    logger = init.logger;
}

void VMeNmpcEngine::setTarget(fp_point2d point) { currentTarget = point; }

up_VMeCommand VMeNmpcEngine::nextCommand() {
  if (machineIsHalted)
    return up_VMeCommand{new VMeStop()};
  else if (cmdsExecutedFromCurrentHorizon++ >= model->getHorizonSize())
    return up_VMeCommand{new VMeNullCmd()};
  else
    return model->getCommand(cmdsExecutedFromCurrentHorizon);
}

void VMeNmpcEngine::seed(xyth pose, fp_point2d target) {
  model->seed(pose, target);
  cmdsExecutedFromCurrentHorizon = 0;
  if (model->getTargetDistance() > targetDistanceTolerance) {
    machineIsHalted = false;
    minimizer->solveOptimalControlHorizon();
    logger->logPositionAndError();
  } else
    machineIsHalted = true;
  notify();
}

void VMeNmpcEngine::seed(xyth pose) {
  model->seed(pose);
  cmdsExecutedFromCurrentHorizon = 0;
  if (model->getTargetDistance() > targetDistanceTolerance) {
    machineIsHalted = false;
    auto minimizerStatus = minimizer->solveOptimalControlHorizon();
    if (minimizerStatus == MinimizerCode::reachedIterationLimit)
      throw MinimizerReachedIterationLimit();
    logger->logPositionAndError();
  } else
    machineIsHalted = true;
  notify();
}

bool VMeNmpcEngine::isHalted() { return machineIsHalted; }

vMeModel* VMeNmpcEngine::_getModelPointer_() { return model; }

NmpcMinimizer* VMeNmpcEngine::_getMinimizerPointer_() { return minimizer; }