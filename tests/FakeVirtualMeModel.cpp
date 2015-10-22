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

#include "FakeVirtualMeModel.hpp"

void FakeVirtualMeModel::recordEvent(char eventCode) {
  eventHistory += eventCode;
}

FakeVirtualMeModel::FakeVirtualMeModel(unsigned N) : N{N} {}

void FakeVirtualMeModel::seed(xyvth) { recordEvent('S'); }

void FakeVirtualMeModel::seed(xyvth position, Point2R target) {
  auto displacement = target - Point2R{position.x, position.y};
  distanceToTarget = std::sqrt(dot(displacement, displacement));
  recordEvent('S');
}

void FakeVirtualMeModel::forecast() { recordEvent('F'); }

void FakeVirtualMeModel::setTrackingErrors() { recordEvent('E'); }

void FakeVirtualMeModel::computePathPotentialGradient(ObstacleStack&) {
  recordEvent('P');
}

void FakeVirtualMeModel::computeGradient() { recordEvent('G'); }

fptype FakeVirtualMeModel::getTargetDistance() {
  recordEvent('D');
  return distanceToTarget;
}

upVirtualMeCommand FakeVirtualMeModel::getCommand(int) {
  recordEvent('C');
  return upVirtualMeCommand{new VMeV{0, 0, 0}};
}

std::string FakeVirtualMeModel::getEventHistory() { return eventHistory; }

unsigned FakeVirtualMeModel::getHorizonSize() const { return N; }