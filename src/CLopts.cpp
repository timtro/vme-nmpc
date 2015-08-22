/*
 * vme-nmpc/src/commandline_options.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2015-08-21
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2015 - Timothy A.V. Teatro
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

#include "CLopts.hpp"

#include <cstdio>
#include <cstdlib>
// #include <cstring>
#include <string>
#include <getopt.h>


CLopts::CLopts(int argc, char** argv)
  : infile{"null"}, host{"localhost"}, port{5010},
    verbose{false}, quiet{false} {

  // char *pvalue = nullptr, *hvalue = nullptr, *fvalue = nullptr;
  // int index;
  int c;

  opterr = 0;

  for(;;) {

    static struct option long_options[] = {
      {"infile",  required_argument, 0, 'f'},
      {"host", required_argument, 0, 'h'},
      {"port", required_argument, 0, 'p'},
      {"verbose", no_argument, 0, 'v'},
      {"quiet", no_argument, 0, 'q'},
      {0, 0, 0, 0}
    };
    /* getopt_long stores the option index here. */
    int option_index = 0;

    c = getopt_long(argc, argv, "qvp:h:f:",
                    long_options, &option_index);
    if(c == -1)
      break;

    switch(c) {
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
      if(optopt == 'p' || optopt == 'h' || optopt == 'f') {
        // std::string err{"Option -"+optopt+" requires an argument"};
        // throw std::runtime_error(err);
      } else if(isprint(optopt)) {
        // std::string err{"Unknown option `-"+optopt+"'"};
        // throw std::runtime_error(err);
      } else {
        // std::string err{"Unknown option character `"+optopt+"'.\n"};
        // throw std::runtime_error(err);
        return;
      }
      break;
    default:
      abort();
    }
  }
}

