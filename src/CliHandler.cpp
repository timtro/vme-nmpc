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

#include "CliHandler.hpp"
#include "ObstacleTypes/PointObstacle.hpp"

#include <unistd.h>

std::string detachToken(std::string&);
std::string fetchMessageString(const int);
void makeLowerCase(std::string&);

CliHandler::CliHandler(TargetContainer* tgt, ObstacleContainer* obs)
    : targets(tgt), obstacles(obs) {}

CliHandler::CliHandler(CliHandler&& source) {
  targets = source.targets;
  source.targets = nullptr;
  obstacles = source.obstacles;
  source.obstacles = nullptr;
}

CliHandler::CliHandler(const CliHandler& source) {
  targets = source.targets;
  obstacles = source.obstacles;
}

void CliHandler::operator()(const int sockfd) {
  for (;;) {
    std::string line = fetchMessageString(sockfd);
    auto cmd = detachToken(line);
    makeLowerCase(cmd);
    // TODO: Add e-stop.
    if (cmd == "at")
      addTarget(line);
    else if (cmd == "ao")
      addObstacle(line);
    else if (cmd == "clear")
      clear(line);
    else if (cmd == "eot")
      return;
  }
}

void CliHandler::addTarget(std::string line) {
  fptype x{0}, y{0}, tol{0};
  try {
    x = std::stof(detachToken(line));
    y = std::stof(detachToken(line));
    tol = std::stof(detachToken(line));
  } catch (std::invalid_argument) {
    return;
  }
  targets->emplace_back(new Target(x, y, tol));
}

void CliHandler::addObstacle(std::string line) {
  auto obstacleType = detachToken(line);
  fptype x{0}, y{0}, pwr{0}, eps{0};
  if (obstacleType == "PointObstacle") {
    try {
      x = std::stof(detachToken(line));
      y = std::stof(detachToken(line));
      pwr = std::stof(detachToken(line));
      eps = std::stof(detachToken(line));
    } catch (std::invalid_argument) {
      return;
    }
    obstacles->pushObstacle(new PointObstacle{fp_point2d{x, y}, pwr, eps});
  }
}

void CliHandler::clear(std::string line) {
  auto whatToClear = detachToken(line);
  if (whatToClear == "targets") targets->clear();
  if (whatToClear == "obstacles") obstacles->clearObstacleContainer();
  if (whatToClear == "all") {
    targets->clear();
    obstacles->clearObstacleContainer();
  }
}

std::string detachToken(std::string& line) {
  unsigned begin = 0;
  while (begin < line.length() && std::isspace(line[begin])) ++begin;
  unsigned end = begin;
  while (end < line.length() && !std::isspace(line[end])) ++end;
  std::string token = line.substr(begin, end - begin);
  line = line.substr(end, line.length() - end);
  return token;
}

std::string fetchMessageString(const int sockfd) {
  char buff[120];
  if (read(sockfd, &buff, 80) < 1) return "eot";
  return std::string{buff};
}

void makeLowerCase(std::string& string) {
  for (auto& each : string) each = std::tolower(each);
}