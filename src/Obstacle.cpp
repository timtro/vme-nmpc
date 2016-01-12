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

#include "linear.hpp"
#include "Obstacle.hpp"

fp_point2d ObstacleContainer::gradPhi(fp_point2d refPoint) {
  std::lock_guard<std::mutex> lock(mutex);
  auto sum = fp_point2d{0.f, 0.f};
  for (auto const& each : obstacles) sum += each->gradPhi(refPoint);
  return sum;
}

void ObstacleContainer::pushObstacleUniquePtr(std::unique_ptr<Obstacle> obs) {
  std::lock_guard<std::mutex> lock(mutex);
  obstacles.push_back(std::move(obs));
}

void ObstacleContainer::pushObstacle(Obstacle* obs) {
  std::lock_guard<std::mutex> lock(mutex);
  obstacles.push_back(std::unique_ptr<Obstacle>{obs});
}

void ObstacleContainer::popObstacle() {
  std::lock_guard<std::mutex> lock(mutex);
  obstacles.pop_back();
}

size_t ObstacleContainer::numberOfObstacles() {
  std::lock_guard<std::mutex> lock(mutex);
  return obstacles.size();
}

void ObstacleContainer::clearObstacleContainer() {
  std::lock_guard<std::mutex> lock(mutex);
  obstacles.clear();
}

bool ObstacleContainer::hasObstacles() {
  std::lock_guard<std::mutex> lock(mutex);
  return !obstacles.empty();
}

std::unique_ptr<Obstacle>& ObstacleContainer::operator[](const int i) {
  return obstacles[i];
}