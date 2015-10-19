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

VirtualMeNmpcEngine::VirtualMeNmpcEngine(NmpcModel &model,
                                         NmpcMinimizer &minimizer)
    : model(model), minimizer(minimizer) {}

void VirtualMeNmpcEngine::setTarget(Point2R point) { currentTarget = point; }

upVirtualMeCommand VirtualMeNmpcEngine::nextCommand() {
  return upVirtualMeCommand{new VMeStop()};
}

void VirtualMeNmpcEngine::seed(xyvth pose, Point2R target) {
  model.seed(pose, target);
  notify();
}

// void VirtualMeNmpcEngine::solveHorizon(xyvth origin) {
//   model.seed(origin);

//   while (model.distanceToTarget() > targetDistanceTolerance_) {
//   }
// }