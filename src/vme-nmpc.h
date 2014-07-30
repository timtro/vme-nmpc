/*
 * vme-nmpc.h
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-10
 *
 * This file is part of vme-nmpc.
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

#include "class_robot.h"
#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"
#include "struct_cl_opts.h"
#include "struct_cmd.h"
#include "inc_errhandler.h"
#include "class_globalPath.h"
#include "class_nmpcEngine.h"
#include "class_SDmin.h"

// from sockcomm.cpp
int init_vme_sock();

// from input.cpp
int parse_command_line( int, char**, robot*, cl_opts* );
void parse_input_file( nmpc&, const char* );

// from time-sync.cpp
double wall_time();

// from output.cpp
void empty_output_hook( const qnu*, const Lagr*, const nmpc& );
void empty_output_hook( const qnu*, const Lagr*, const nmpc&, const float* );
void empty_output_hook( const unsigned int*, const double* );
void print_greeting( const nmpc& );
void print_pathnerr( const qnu*, const Lagr*, const nmpc& );
void print_LG( const qnu*, const Lagr*, const nmpc&, const float* );
void print_SD( const unsigned int*, const double* );
void print_TR( const unsigned int*, const double* );