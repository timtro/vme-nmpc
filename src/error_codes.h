/*
 * error_codes.h
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

#ifndef __ERROR_CODES_H__
#define __ERROR_CODES_H__

#define SOCK_CANNOT_CREATE_SOCK          0x11
#define SOCK_CANNOT_CONNECT              0x12
#define CL_NO_ARG                        0x13
#define CL_INVALID_OPT                   0x14
#define CL_UNKOWN_OPT_CHAR               0x15
#define SOCK_READ_ERROR                  0x16
#define SOCK_WRITE_ERROR                 0x17
#define CANNOT_OPEN_INFILE               0x18
#define CANNOT_READ_INFILE               0x19
#define INVALID_INPUT_FILE_SYNTAX        0x1A
#define RECOVERABLE_INPUT_FILE_SYNTAX    0x1B
#define FATAL_INPUT_FILE_SYNTAX          0x1C
#define ODD_NUMBER_TGT_COORDINATES       0x1D
#define ODD_NUMBER_OBST_COORDINATES      0x1E
#define FATAL_INPUT_FILE_ERROR           0x1F
#define EXCEEDED_MAX_SD_ITER             0x20
#define TRAPPED_IN_NMPC_LOOP             0x21

#endif
