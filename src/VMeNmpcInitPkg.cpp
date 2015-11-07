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
  if ((minimizer.get() != nullptr) || (logger.get() != nullptr))
    throw ModelMustBeInitializedBeforeMinimizerOrLogger();
  else if (model.get() != nullptr)
    throw InitPkgCanOnlyBeUsedOnceToInitializeAModel();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(
    NmpcModel<xyth, fp_point2d, up_VMeCommand>* caller) {
  model = std::unique_ptr<NmpcModel<xyth, fp_point2d, up_VMeCommand>>(caller);
}

bool VMeNmpcInitPkg::minimizerBindingSafetyCheck() {
  if (model.get() == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer.get() != nullptr)
    throw InitPkgAlreadyHasBoundMinimizer();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(NmpcMinimizer* caller) {
  minimizer = std::unique_ptr<NmpcMinimizer>(caller);
}

bool VMeNmpcInitPkg::loggerBindingSafetyCheck() {
  if (model.get() == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer.get() == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
  else
    return true;
}

void VMeNmpcInitPkg::bindIntoAggregator(VMeLogger* caller) {
  logger = std::unique_ptr<VMeLogger>(caller);
}

bool VMeNmpcInitPkg::aggregatorCompletionSafetyCheck() {
  if (model.get() == nullptr)
    throw InitPkgDoesNotContainPointerToAModel();
  else if (minimizer.get() == nullptr)
    throw InitPkgDoesNotContainPointerToAMinimizer();
  else
    return true;
};