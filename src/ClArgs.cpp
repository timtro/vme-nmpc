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

#include "ClArgs.hpp"

#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <getopt.h>

ClArgs::ClArgs(int argc, char** argv)
    : infile{"null"},
      host{"localhost"},
      port{5010},
      verbose{false},
      quiet{false} {
  int c;

  char errNote[256];
  opterr = 0;

  for (;;) {
    static struct option long_options[] = {
        {"infile", required_argument, 0, 'f'},
        {"host", required_argument, 0, 'h'},
        {"port", required_argument, 0, 'p'},
        {"verbose", no_argument, 0, 'v'},
        {"quiet", no_argument, 0, 'q'},
        {0, 0, 0, 0}};
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long(argc, argv, "qvp:h:f:", long_options, &option_index);
    if (c == -1) break;

    switch (c) {
      case 'p':
        port = std::stoi(optarg);
        break;
      case 'h':
        host = std::string{optarg};
        break;
      case 'f':
        infile = std::string{optarg};
        break;
      case 'v':
        verbose = true;
        break;
      case 'q':
        quiet = true;
        break;
      case '?':
        if (optopt == 'p' || optopt == 'h' || optopt == 'f') {
          sprintf(errNote, "Option -%c: requires an argument.\n", optopt);
          throw std::runtime_error(errNote);
        } else if (isprint(optopt)) {
          sprintf(errNote, "Unknown option `-%c'.\n", optopt);
          throw std::runtime_error(errNote);
        } else {
          sprintf(errNote, "Unknown option character `\\x%x'.\n", optopt);
          throw std::runtime_error(errNote);
          return;
        }
        break;
      default:
        abort();
    }
  }
}
