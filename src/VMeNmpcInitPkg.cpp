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

#include "VMeNmpcInitPkg.hpp"

bool VMeNmpcInitPkg::modelBindingSafetyCheck() {
  if ((minimizer != nullptr) || (logger != nullptr))
    throw ModelMustBeInitializedBeforeMinimizerOrLogger();
  else if (model != nullptr)
    throw InitPkgCanOnlyBeUsedOnceToInitializeAModel();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(
    NmpcModel<xyth, fp_point2d, up_VMeCommand>* caller) {
  model = caller;
}

bool VMeNmpcInitPkg::minimizerBindingSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer != nullptr)
    throw InitPkgAlreadyHasBoundMinimizer();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(NmpcMinimizer* caller) {
  minimizer = caller;
}

bool VMeNmpcInitPkg::loggerBindingSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(VMeLogger* caller) {
  logger = caller;
}

bool VMeNmpcInitPkg::aggregatorCompletionSafetyCheck() {
  if (model == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
  else
    return true;
};