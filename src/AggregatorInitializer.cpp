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

#include "AggregatorInitializer.hpp"

using vMeModelType = NmpcModel<SeedPackage, up_VMeCommand>;

AggregatorInitializer::AggregatorInitializer() {
  defaultParameters_ = std::make_unique<InputFileData>();
  parameters = defaultParameters_.get();
}

AggregatorInitializer::AggregatorInitializer(InputFileData& in)
    : parameters(&in) {}

void AggregatorInitializer::modelBindingSafetyCheck() {
  if ((minimizer != nullptr) || (logger != nullptr))
    throw ModelMustBeInitializedBeforeMinimizerOrLogger();
  else if (model != nullptr)
    throw InitPkgCanOnlyBeUsedOnceToInitializeAModel();
}

void AggregatorInitializer::bindIntoAggregator(vMeModelType* caller) {
  model = caller;
}

void AggregatorInitializer::minimizerBindingSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer != nullptr)
    throw InitPkgAlreadyHasBoundMinimizer();
}

void AggregatorInitializer::bindIntoAggregator(NmpcMinimizer* caller) {
  minimizer = caller;
}

void AggregatorInitializer::loggerBindingSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
}

void AggregatorInitializer::bindIntoAggregator(VMeLogger* caller) {
  logger = caller;
}

void AggregatorInitializer::bindIntoAggregator(
    PathPlanner<SeedPackage>* caller) {
  planner = caller;
}

void AggregatorInitializer::aggregatorCompletionSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
  else if (planner == nullptr)
    throw InitPkgDoesNotContainPointerToAPathPlanner();
}

unsigned AggregatorInitializer::get_nmpcHorizon() {
  return parameters->nmpcHorizon;
}
fptype AggregatorInitializer::get_timeInterval() {
  return parameters->timeInterval;
}
fptype AggregatorInitializer::get_cruiseSpeed() {
  return parameters->cruiseSpeed;
}
fptype AggregatorInitializer::get_Q() { return parameters->Q; }
fptype AggregatorInitializer::get_Q0() { return parameters->Q0; }
fptype AggregatorInitializer::get_R() { return parameters->R; }
fptype AggregatorInitializer::get_sdStepFactor() {
  return parameters->sdStepFactor;
}
fptype AggregatorInitializer::get_sdConvergenceTolerance() {
  return parameters->sdConvergenceTolerance;
}
unsigned AggregatorInitializer::get_maxSdSteps() {
  return parameters->maxSdSteps;
}
fptype AggregatorInitializer::get_targetDistanceTolerance() {
  return parameters->targetDistanceTolerance;
}
std::string AggregatorInitializer::get_jsonLogPath() {
  return parameters->jsonLogPath;
};