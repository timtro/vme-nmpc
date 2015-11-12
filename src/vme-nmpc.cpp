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

// #define BOOST_LOG_DYN_LINK

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <tuple>

#include <boost/program_options.hpp>
#include <boost/log/trivial.hpp>
#include <unistd.h>

#include "Daemon.hpp"
#include "Nav2Robot.hpp"
#include "ClArgs.hpp"
#include "InputFileData.hpp"
#include "VMeNmpcEngine.hpp"
#include "NmpcModels/VMeModel.hpp"
#include "NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "Loggers/JsonLogger.hpp"

// TODO(T.T.): Use Boost property_tree and JSON as input files.
// TODO(T.T.): Use Boost scoped threads that assure that all paths out of a
//             make the thread unjoinable. (As per the advice of Scott Mayers)

// namespace logging = boost::log;

using std::unique_ptr;
using std::make_unique;

void request_handler(int sockfd) {
  char buff[80];

  if (read(sockfd, &buff, 80) < 1) return;

  if (strstr(buff, "exit")) std::exit(0);

  printf("Recieved Message: %s", buff);
  return;
}

// // Consider composing the Initializer with InputFileData
// AggregatorInitializer makeInitializerFromInputData(InputFileData& src) {
//   AggregatorInitializer init;
//   init.nmpcHorizon = src.nmpcHorizon;
//   init.timeInterval = src.timeInterval;
//   init.cruiseSpeed = src.cruiseSpeed;
//   init.Q = src.Q;
//   init.Q0 = src.Q0;
//   init.R = src.R;
//   init.sdStepFactor = src.sdStepFactor;
//   init.sdConvergenceTolerance = src.sdConvergenceTolerance;
//   init.maxSdSteps = src.maxSdSteps;
//   init.targetDistanceTolerance = src.targetDistanceTolerance;
//   return init;
// }

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

  // Daemon command_server(5111, request_handler);

  vme.originate();
  vme.fd(1);

  float x, y, n;
  int q;
  std::tie(x, y, n, q) = vme.q();
  printf("%f %f %f %d\n", x, y, n, q);

  std::string logFilePath{"here.txt"};


  AggregatorInitializer init(inputData);
  auto model = make_unique<VMeModel>(init);
  auto minimizer = make_unique<VMeNaiveSdMinimizer>(init);
  auto logger = make_unique<JsonLogger>(init, logFilePath);
  auto engine = make_unique<VMeNmpcEngine>(init);

  // for(;;) {
  //   std::this_thread::sleep_for(std::chrono::seconds(10));
  // }

  printf("Shutting down!\n");
  fflush(stdout);
  return 0;
}