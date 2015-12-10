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

#include "Target.hpp"

using std::unique_ptr;
using std::make_unique;

void TargetContainer::push_back(unique_ptr<Target> tgt) {
  targets.push_back(std::move(tgt));
}

void TargetContainer::push_front(unique_ptr<Target> tgt) {
  targets.push_front(std::move(tgt));
}

void TargetContainer::pop_back() {
  targets.pop_back();
}
void TargetContainer::pop_front() {
  targets.pop_front();
}

Target& TargetContainer::operator[](const int i) {
  return *targets[i];
}

bool TargetContainer::hasTargets() {
  return !targets.empty();
}

bool TargetContainer::empty() const noexcept {
  return targets.empty();
}

void TargetContainer::emplace_back(Target* tgt) {
  targets.emplace_back(tgt);
}

void TargetContainer::clear() {
  targets.clear();
}