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

#include "VirtualMeNmpcEngine.hpp"
#include "NmpcModel.hpp"

VirtualMeNmpcEngine::VirtualMeNmpcEngine(
    std::unique_ptr<NmpcModel<xyth, fp_point2d, up_VirtualMeCommand>> model,
    std::unique_ptr<NmpcMinimizer> minimizer)
    : model{std::move(model)}, minimizer{std::move(minimizer)} {
  logger = std::unique_ptr<VirtualMeLogger>{new VirtualMeLogger};
}

VirtualMeNmpcEngine::VirtualMeNmpcEngine(
    std::unique_ptr<NmpcModel<xyth, fp_point2d, up_VirtualMeCommand>> model,
    std::unique_ptr<NmpcMinimizer> minimizer,
    std::unique_ptr<VirtualMeLogger> logger)
    : model{std::move(model)},
      minimizer{std::move(minimizer)},
      logger{std::move(logger)} {}

void VirtualMeNmpcEngine::setTarget(fp_point2d point) { currentTarget = point; }

up_VirtualMeCommand VirtualMeNmpcEngine::nextCommand() {
  if (machineIsHalted)
    return up_VirtualMeCommand{new VMeStop()};
  else if (cmdsExecutedFromCurrentHorizon++ >= model->getHorizonSize())
    return up_VirtualMeCommand{new VMeNullCmd()};
  else
    return model->getCommand(cmdsExecutedFromCurrentHorizon);
}

void VirtualMeNmpcEngine::seed(xyth pose, fp_point2d target) {
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

void VirtualMeNmpcEngine::seed(xyth pose) {
  model->seed(pose);
  cmdsExecutedFromCurrentHorizon = 0;
  if (model->getTargetDistance() > targetDistanceTolerance) {
    machineIsHalted = false;
    minimizer->solveOptimalControlHorizon();
    logger->logPositionAndError();
  } else
    machineIsHalted = true;
  notify();
}

bool VirtualMeNmpcEngine::isHalted() { return machineIsHalted; }

vMeModel* VirtualMeNmpcEngine::getModelPointer() { return model.get(); }

NmpcMinimizer* VirtualMeNmpcEngine::getMinimizerPointer() {
  return minimizer.get();
}