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

void PathPlanner::pushFinalTarget(Target tgt) {
  targets.push_back(tgt);
}

void PathPlanner::pushCurrentTarget(Target tgt) {
  targets.push_front(tgt);
}

void PathPlanner::popFinalTarget() {
  targets.pop_back();
}

int PathPlanner::numberOfTargets() {
  return targets.size();
}

Target& PathPlanner::currentTarget() {
  return targets.front();
}

Target& PathPlanner::finalTarget() {
  return targets.back();
}

void PathPlanner::clearTargetList() {
  targets.clear();
}

bool PathPlanner::areTargetsRemaining() {
  return !targets.empty();
}