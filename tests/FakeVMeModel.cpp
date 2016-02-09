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

FakeVMeModel::FakeVMeModel(AggregatorInitializer &init,
                           std::string &historyString)
    : eventHistory(historyString), N(init.get_nmpcHorizon()) {
  init.bind_into_aggregator(this);
}

void FakeVMeModel::record_event(char eventCode) const {
  eventHistory += eventCode;
}

unsigned FakeVMeModel::get_horizonSize() const { return N; }

void FakeVMeModel::seed(SeedPackage &) { record_event('S'); }

void FakeVMeModel::compute_forecast() noexcept { record_event('F'); }

void FakeVMeModel::compute_tracking_errors() noexcept { record_event('E'); }

void FakeVMeModel::compute_path_potential_gradient(
    ObstacleContainer &) noexcept {
  record_event('P');
}

void FakeVMeModel::compute_gradient() noexcept { record_event('G'); }

up_VMeCommand FakeVMeModel::retrieve_command(int) const {
  record_event('C');
  return up_VMeCommand{new VMeV{0, 0, 0}};
}

std::string FakeVMeModel::get_eventHistory() const { return eventHistory; }
