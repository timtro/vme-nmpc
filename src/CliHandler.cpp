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

CliHandler::CliHandler(std::deque<Target*>* tgt, ObstacleContainer* obs)
    : targets(tgt), obstacles(obs) {}

CliHandler::CliHandler(CliHandler&& old) {
  targets = old.targets;
  old.targets = nullptr;
  obstacles = old.obstacles;
  old.obstacles = nullptr;
}

CliHandler::CliHandler(const CliHandler& old) {
  targets = old.targets;
  obstacles = old.obstacles;
}

void CliHandler::operator()(const int sockfd) {
  std::string line = fetchMessageString(sockfd);
  auto cmd = detachToken(line);
  makeLowerCase(cmd);
  if (cmd == "at")
    addTarget(line);
  else if (cmd == "ao")
    addObstacle(line);
}

void CliHandler::addTarget(std::string line) {
  targets->emplace_back(new Target(1, 1, .1));
}

void CliHandler::addObstacle(std::string line) {
  auto obstacleType = detachToken(line);
  if (obstacleType == "PointObstacle") {
    fptype x = std::stof(detachToken(line));
    fptype y = std::stof(detachToken(line));
    fptype pwr = std::stof(detachToken(line));
    fptype eps = std::stof(detachToken(line));
    obstacles->pushObstacle(new PointObstacle{fp_point2d{x, y}, pwr, eps});
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
  if (read(sockfd, &buff, 80) < 1) return "";
  return std::string{buff};
}

void makeLowerCase(std::string& s) {
  for (auto& elem : s) elem = std::tolower(elem);
}