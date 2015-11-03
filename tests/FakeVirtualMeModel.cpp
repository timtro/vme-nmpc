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

void FakeVirtualMeModel::recordEvent(char eventCode) const {
  eventHistory += eventCode;
}

FakeVirtualMeModel::FakeVirtualMeModel(unsigned N) : N{N} {}

unsigned FakeVirtualMeModel::getHorizonSize() const { return N; }

fptype FakeVirtualMeModel::getTargetDistance() const noexcept {
  recordEvent('D');
  return distanceToTarget;
}

void FakeVirtualMeModel::seed(xyth) { recordEvent('S'); }

void FakeVirtualMeModel::seed(xyth position, fp_point2d target) {
  auto displacement = target - fp_point2d{position.x, position.y};
  distanceToTarget = std::sqrt(dot(displacement, displacement));
  recordEvent('S');
}

void FakeVirtualMeModel::computeForecast() noexcept { recordEvent('F'); }

void FakeVirtualMeModel::computeTrackingErrors() noexcept { recordEvent('E'); }

void FakeVirtualMeModel::computePathPotentialGradient(
    ObstacleContainer&) noexcept {
  recordEvent('P');
}

void FakeVirtualMeModel::computeGradient() noexcept { recordEvent('G'); }

up_VirtualMeCommand FakeVirtualMeModel::getCommand(int) const {
  recordEvent('C');
  return up_VirtualMeCommand{new VMeV{0, 0, 0}};
}

std::string FakeVirtualMeModel::getEventHistory() const { return eventHistory; }