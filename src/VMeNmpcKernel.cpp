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
  init.aggregator_completion_safety_check();
  model = init.model;
  minimizer = init.minimizer;
  planner = init.planner;

  cmdsExecutedFromCurrentHorizon = model->get_horizonSize();

  if (init.logger == nullptr) {
    noOpLogger = std::make_unique<VMeLogger>();
    logger = noOpLogger.get();
  } else
    logger = init.logger;

  logger->log_constants(init);
}

up_VMeCommand VMeNmpcKernel::next_command() {
  if (cmdsExecutedFromCurrentHorizon++ >= model->get_horizonSize())
    return up_VMeCommand{new VMeNullCmd()};
  else
    return model->retrieve_command(cmdsExecutedFromCurrentHorizon);
}

void VMeNmpcKernel::seed(SeedPackage& seed) {
  model->seed(seed);
  cmdsExecutedFromCurrentHorizon = model->get_horizonSize();
}

void VMeNmpcKernel::nmpc_step(SeedPackage& seed) {
  cmdsExecutedFromCurrentHorizon = 0;
  model->seed(seed);
  auto minimizerStatus = minimizer->solve_optimal_control_horizon();
  if (minimizerStatus == MinimizerCode::reachedIterationLimit)
    throw MinimizerReachedIterationLimit();
  logger->log_model_state();
  logger->log_minimizer_state();
  notify();
}

void VMeNmpcKernel::run() {
  do {
    nmpc_step(planner->get_seed());
  } while (planner->is_continuing());
}
