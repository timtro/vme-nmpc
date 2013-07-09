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
#include "inc_errhandler.h"

// from sockcomm.cpp
int init_vme_sock();

// from input.cpp
int parse_command_line( int, char**, robot*, cl_opts* );
void parse_input_file( nmpc&, const char* );

// from nmpc-funcs.cpp
void init_qu_and_p( qnu*, Lagr*, nmpc& );
void get_gradient( qnu*, Lagr*, nmpc&, float* );
float predict_horizon( qnu*, Lagr*, const nmpc& );
float costfun( const qnu*, const Lagr*, const nmpc& );
void swap_fptr( float**, float** );

// from time-sync.cpp
double wall_time();

// from output.cpp
void empty_output_hook( const qnu*, const Lagr*, const nmpc& );
void empty_output_hook( const qnu*, const Lagr*, const nmpc&, const float* );
void print_greeting( const nmpc& );
void print_pathnerr( const qnu*, const Lagr*, const nmpc& );
void print_lagrange_grad( const qnu*, const Lagr*, const nmpc&, const float* );
