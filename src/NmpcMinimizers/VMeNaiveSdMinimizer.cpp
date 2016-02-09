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

#include "VMeNaiveSdMinimizer.hpp"
#include "../Obstacle.hpp"

VMeNaiveSdMinimizer::VMeNaiveSdMinimizer(AggregatorInitializer& init)
    : maxSdSteps(init.get_maxSdSteps()),
      sdStepFactor(init.get_sdStepFactor()),
      sdConvergenceTolerance(init.get_sdConvergenceTolerance()) {
  init.minimizer_binding_safety_check();
  model = dynamic_cast<VMeModel*>(init.model);
  obstacles = init.obstacles;
  init.bind_into_aggregator(this);
}

MinimizerCode VMeNaiveSdMinimizer::solve_optimal_control_horizon() noexcept {
  sdLoopCount = 0;
  status = MinimizerCode::active;
  do {
    model->compute_forecast();
    model->compute_tracking_errors();
    model->compute_path_potential_gradient(*obstacles);
    model->compute_gradient();
    ++sdLoopCount;
  } while (iterate());

  lastSdLoopCount = sdLoopCount;
  return status;
}

bool VMeNaiveSdMinimizer::iterate() noexcept {
  model->Dth -= sdStepFactor * model->grad;
  if (model->gradNorm < sdConvergenceTolerance) {
    status = MinimizerCode::success;
    return false;
  } else if (sdLoopCount >= maxSdSteps) {
    status = MinimizerCode::reachedIterationLimit;
    return false;
  } else
    return true;
}
