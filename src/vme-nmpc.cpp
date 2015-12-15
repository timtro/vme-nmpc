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
#include <tuple>

#include "Daemon.hpp"
#include "Nav2Robot.hpp"
#include "ClArgs.hpp"
#include "CliHandler.hpp"
#include "InputFileData.hpp"
#include "Loggers/JsonLogger.hpp"
#include "NmpcModels/VMeModel.hpp"
#include "NmpcMinimizers/VMeNaiveSdMinimizer.hpp"
#include "PathPlanners/VMePathPlanner.hpp"
#include "VMeNmpcKernel.hpp"
#include "Executors/VMeDefaultExecutor.hpp"
#include "trig.hpp"

// TODO(T.T.): Use Boost scoped threads that assure that all paths out of a
//             make the thread unjoinable. (As per the advice of Scott Mayers)

using std::unique_ptr;
using std::make_unique;

int main(int argc, char** argv) {
  ClArgs cmdlnArgs(argc, argv);
  InputFileData inputFileData;
  inputFileData.load(cmdlnArgs.infile);

  Nav2Robot vme(cmdlnArgs.host, cmdlnArgs.port);

  try {
    vme.connect();
  } catch (const std::exception& ex) {
    printf("Errer while trying to connect to Nav2 device.\n Error message: ");
    printf("%s\n", ex.what());
    std::exit(1);
  }

  ObstacleContainer obstacles;
  TargetContainer targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};

  Daemon command_server(5111, commandHandler);
  vme.originate();

  AggregatorInitializer init(inputFileData);
  init.targets = &targets;
  init.obstacles = &obstacles;
  auto model = make_unique<VMeModel>(init);
  auto minimizer = make_unique<VMeNaiveSdMinimizer>(init);
  auto planner = make_unique<VMePathPlanner>(init);
  auto logger = make_unique<JsonLogger>(init, inputFileData.jsonLogPath);
  auto kernel = make_unique<VMeNmpcKernel>(init);

  auto executor = make_unique<VMeDefaultExecutor>(kernel.get());

  planner->set_stateEstimateRetriever([&vme]() {
    SeedPackage state;
    int q;
    std::tie(state.pose.x, state.pose.y, state.pose.th, q) = vme.q();
    state.pose.th = degToRad(state.pose.th);
    return state;
  });

  for (;;) {
    while (targets.empty()) {
      printf("WAITING(1)\n");
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      kernel->nmpcStep(planner->getSeed());
      executor->run(vme);
    } while (planner->isContinuing());
    vme.stop();
  }

  printf("Shutting down!\n");
  fflush(stdout);
  return 0;
}