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

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <locale>  // std::locale, std::tolower
#include <string>
#include <tuple>

#include <unistd.h>

#include "Daemon.hpp"
#include "Nav2Robot.hpp"
#include "ClArgs.hpp"
#include "InputFileData.hpp"
#include "VMeNmpcEngine.hpp"
#include "PathPlanner.hpp"
#include "NmpcModels/VMeModel.hpp"
#include "NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "Loggers/JsonLogger.hpp"

// TODO(T.T.): Use Boost scoped threads that assure that all paths out of a
//             make the thread unjoinable. (As per the advice of Scott Mayers)

using std::unique_ptr;
using std::make_unique;

std::string detachToken(std::string& line) {
  unsigned begin = 0;
  while (begin < line.length() && std::isspace(line[begin])) ++begin;
  unsigned end = begin;
  while (end < line.length() && !std::isspace(line[end])) ++end;
  std::string token = line.substr(begin, end - begin);
  line = line.substr(end, line.length() - end);
  return token;
}

std::string fetchMessageString(int sockfd) {
  char buff[80];
  if (read(sockfd, &buff, 80) < 1) return "";
  return std::string{buff};
}

void makeLowerCase(std::string& s) {
  for (auto& elem : s) elem = std::tolower(elem);
  // std::transform(s.begin(), s.end(), s.begin(), std::tolower);
}

struct CLIExecutor {
  TargetStack* targets{nullptr};
  ObstacleContainer* obstacles{nullptr};

  void operator()(int sockfd) {
    std::string line = fetchMessageString(sockfd);
    auto cmd = detachToken(line);
    makeLowerCase(cmd);
    if (cmd == "at") {
      std::cout << "GOT AT!" << std::endl;
      // targets->pushFinalTarget(Target{1, 1, 1});
    }
  }
};

int main(int argc, char** argv) {
  ClArgs cmdlnArgs(argc, argv);
  InputFileData inputData;
  inputData.load(cmdlnArgs.infile);

  Nav2Robot vme(cmdlnArgs.host, cmdlnArgs.port);

  try {
    vme.connect();
  } catch (const std::exception& ex) {
    printf("Errer while trying to connect to Nav2 device.\n Error message: ");
    printf("%s\n", ex.what());
    std::exit(1);
  }

  ObstacleContainer obstacles;
  TargetStack targets;
  std::function<void(int)> commandHandler = CLIExecutor();

  Daemon command_server(5111, commandHandler);

  vme.originate();
  vme.fd(1);

  float x, y, n;
  int q;
  std::tie(x, y, n, q) = vme.q();
  printf("%f %f %f %d\n", x, y, n, q);

  AggregatorInitializer init(inputData);
  auto model = make_unique<VMeModel>(init);
  auto minimizer = make_unique<VMeNaiveSdMinimizer>(init);
  auto logger = make_unique<JsonLogger>(init, inputData.jsonLogPath);
  auto engine = make_unique<VMeNmpcEngine>(init);

  for (;;) {
    std::this_thread::sleep_for(std::chrono::seconds(10));
    // test.engine->seed(xyth{0, 0, 0}, fp_point2d{3, 4});
    // while (isMoveCmd(exec.commandFromLastNotify.get())) {
    //   test.engine->seed(xyth{
    //       test.model->get_x()[1], test.model->get_y()[1],
    //       test.model->get_th()[1],
    //   });
    }

    printf("Shutting down!\n");
    fflush(stdout);
    return 0;
  }