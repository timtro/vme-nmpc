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

// void throwIfInitPkgHasNoModel(VMeNmpcInitPkg& init) {
//   if (!init._hasInitializedModel_ || (init.model.get() == nullptr))
//     throw InitPkgDoesNotContainPointerToAModelForLogger();
// }

// void throwIfInitPkgHasNoMinimizer(VMeNmpcInitPkg& init) {
//   if (!init._hasInitializedMinimizer_ || (init.minimizer.get() == nullptr))
//     throw InitPkgDoesNotContainPointerToAMinimizerForLogger();
// }

#include "VMeNmpcInitPkg.hpp"

bool VMeNmpcInitPkg::bindToModel(NmpcModel<xyth, fp_point2d, up_VMeCommand>* caller) {
  if (_hasInitializedMinimizer_ || _hasInitializedLogger_)
    throw ModelMustBeInitializedBeforeMinimizerOrLogger();
  else if (_hasInitializedModel_)
    throw InitPkgCanOnlyBeUsedOnceToInitializeAModel();
  else {
    _hasInitializedModel_ = true;
    model = std::unique_ptr<NmpcModel<xyth, fp_point2d, up_VMeCommand>>(caller);
    return true;
  }
}

bool VMeNmpcInitPkg::bindToMinimizer(NmpcMinimizer* caller) {
  if (!_hasInitializedModel_ || (model.get() == nullptr))
    throw InitPkgDoesNotContainPointerToAModelForMinimizer();
  else if (_hasInitializedMinimizer_ || (minimizer.get() != nullptr))
    throw InitPkgCanOnlyBeUsedOnceToInitializeASingleMinimizer();
  else {
    _hasInitializedMinimizer_ = true;
    minimizer = std::unique_ptr<NmpcMinimizer>(caller);
    return true;
  }
}

bool VMeNmpcInitPkg::bindToLogger(VMeLogger* caller) {
  if (!_hasInitializedModel_ || (model.get() == nullptr))
    throw InitPkgDoesNotContainPointerToAModelForLogger();
  else if (!_hasInitializedMinimizer_ || (minimizer.get() == nullptr))
    throw InitPkgDoesNotContainPointerToAMinimizerForLogger();
  else {
    _hasInitializedLogger_ = true;
    logger = std::unique_ptr<VMeLogger>(caller);
    return true;
  }
}