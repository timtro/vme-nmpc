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

#include "FakeVMeModel.hpp"

FakeVMeModel::FakeVMeModel(AggregatorInitializer& init, std::string& historyString)
    : eventHistory(historyString), N(init.get_nmpcHorizon()) {
    init.bindIntoAggregator(this);
}

void FakeVMeModel::recordEvent(char eventCode) const {
  eventHistory += eventCode;
}

unsigned FakeVMeModel::get_horizonSize() const { return N; }

fptype FakeVMeModel::getTargetDistance() const noexcept {
  recordEvent('D');
  return distanceToTarget;
}

void FakeVMeModel::seed(SeedPackage&) { recordEvent('S'); }

void FakeVMeModel::computeForecast() noexcept { recordEvent('F'); }

void FakeVMeModel::computeTrackingErrors() noexcept { recordEvent('E'); }

void FakeVMeModel::computePathPotentialGradient(
    ObstacleContainer&) noexcept {
  recordEvent('P');
}

void FakeVMeModel::computeGradient() noexcept { recordEvent('G'); }

up_VMeCommand FakeVMeModel::retrieveCommand(int) const {
  recordEvent('C');
  return up_VMeCommand{new VMeV{0, 0, 0}};
}

std::string FakeVMeModel::getEventHistory() const { return eventHistory; }