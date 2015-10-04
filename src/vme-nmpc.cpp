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

#include "vme-nmpc.h"
#include "Daemon.hpp"
#include "Nav2Robot.hpp"
#include "ClArgs.hpp"
#include "InputFileData.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <tuple>

#include <boost/program_options.hpp>

#include <unistd.h>

// TODO(T.T.): Use Boost property_tree and JSON as input files.
// TODO(T.T.): Use Boost scoped threads that assure that all paths out of a
//             make the thread unjoinable. (As per the advice of Scott Mayers)


void request_handler(int sockfd) {

  char buff[80];

  if (read(sockfd, &buff, 80) < 1) return;

  if (strstr(buff, "exit")) std::exit(0);

  printf("Recieved Message: %s", buff);
  return;
}

int main(int argc, char** argv) {

  ClArgs cmdln_args(argc, argv);

  InputFileData input_data;
  input_data.load(cmdln_args.infile);

  Nav2Robot vme(cmdln_args.host, cmdln_args.port);

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
  std::tie(x,y,n,q) = vme.q();
  printf("%f %f %f %d\n", x,y,n,q);
  // for(;;) {
  //   std::this_thread::sleep_for(std::chrono::seconds(10));
  // }

  printf("Shutting down!\n");
  fflush(stdout);
  return 0;

}