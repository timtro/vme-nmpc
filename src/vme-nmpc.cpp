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
#include "VMeNmpcKernel.hpp"

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
  std::deque<Target*> targets;
  std::function<void(int)> commandHandler{CliHandler(&targets, &obstacles)};

  Daemon command_server(5111, commandHandler);

  vme.originate();
  vme.fd(1);

  float x, y, n;
  int q;
  std::tie(x, y, n, q) = vme.q();
  printf("%f %f %f %d\n", x, y, n, q);

  AggregatorInitializer init(inputFileData);
  auto model = make_unique<VMeModel>(init);
  auto minimizer = make_unique<VMeNaiveSdMinimizer>(init);
  auto logger = make_unique<JsonLogger>(init, inputFileData.jsonLogPath);
  auto engine = make_unique<VMeNmpcKernel>(init);

  for (;;) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
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