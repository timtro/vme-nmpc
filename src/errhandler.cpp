/*
 * errhandler.c
 * This file is part of vme-nmpc
 *
 * Copyright (C) 2013 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <stdlib.h>

#include "error_codes.h"

void report_error( const int errno, const char *notes )
  {
    switch ( errno )
      {
    case SOCK_CANNOT_CREATE_SOCK :
      fprintf(stderr, "\n[0x%2x] Exit. Could not create socket\n", errno);
      exit(EXIT_FAILURE);
    case SOCK_CANNOT_CONNECT :
      fprintf(stderr, "\n[0x%2x] Exit. Could not connect to server\n", errno);
      exit(EXIT_FAILURE);
    case RECOVERABLE_INPUT_FILE_SYNTAX :
      fprintf(stderr,
              "\n[0x%2x] Warning: Recoverable syntax error\
 in input file.\n",
              errno);
      fprintf(stderr, "%s\n\n", notes);
      fflush(stderr);
      break;
    case FATAL_INPUT_FILE_SYNTAX :
      fprintf(stderr,
              "\n[0x%2x] Fatal: Unrecoverable syntax error while\
 reading input file.\n",
              errno);
      fprintf(stderr, "%s\n\n", notes);
      fflush(stderr);
    default :
      fprintf(stderr, "[0x%2x] Exit. Unknown Error\n", errno);
      printf("# ERR: [0x%2x]\n", errno);
      if ( notes != NULL )
        {
          fprintf(stderr, "Additional info: ");
          fputs(notes, stderr);
          fprintf(stderr, "\n\n");
        }
      fflush(stderr);
      fflush(stdout);
      exit(EXIT_FAILURE);
      }
  }
