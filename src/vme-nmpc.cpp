/*
 * vme-nmpc.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-10
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013--2014 - Timothy A.V. Teatro
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

#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <unistd.h>

#include "vme-nmpc.h"

#define MAX_SD_ITER 25000
#define MAX_NMPC_ITER 3000

#define SEC_TO_USEC 1e-6

int main( int argc, char **argv )
{

	void
	( *hook_print_SE )( const qnu*, const Lagr*, const nmpc& ) = NULL;
	void
	( *hook_print_LG )( const qnu*, const Lagr*, const nmpc&, const float* ) = NULL;
	void
	( *hook_print_SD )( const unsigned int*, const double* ) = NULL;
	void
	( *hook_print_TR )( const unsigned int*, const double* ) = NULL;

	char errnote[256];
	unsigned int k = 0, current_tgt_no = 0;
	robot vme;
	double sd_loop_time, time_last_cmd_sent = 0, now;
	nmpc C;
	cl_opts clopts = {false, false, false, false,
	                  false, false, false, false, "\0"
	                 };
	parse_command_line( argc, argv, &vme, &clopts );
	parse_input_file( C, vme.conffile() );
	print_greeting( C );

	qnu* qu = ( qnu* ) calloc( C.N, sizeof( qnu ) );
	Lagr* p = ( Lagr* ) calloc( C.N, sizeof( Lagr ) );
	nmpcEngine nmpc;
	double* time_to_tgt = ( double* ) calloc( C.ntgt, sizeof( float ) );
	C.cur_tgt = C.tgt;
	C.control_step = 0;
	float trip_cost = 0.;
	optimizer opt ( qu, p, C, MAX_SD_ITER );
	float* optimizer::*thegrad = &optimizer::grad; // pointer to grad from opt so I can pass it.
	nmpc.init_qu_and_p( qu, p, C );
	nmpc.tgtdist = C.tgttol + 1; // Just to get us into the waypoint loop.
	C.horizon_loop = 0;

	/*
	 * This next block of decisions sort out the hooks used to print output.
	 * Rather than include the ifs in each loop, I'm using function pointers
	 * which are set at runtime based on CL flags for verbosity.
	 */
	if ( clopts.selec_verbose )
	{
		hook_print_SE = &print_pathnerr;
		hook_print_LG = &print_LG;
		hook_print_SD = &print_SD;
		hook_print_TR = &print_TR;
	}
	else if ( clopts.selec_verbose )
	{
		hook_print_SE = &empty_output_hook;
		hook_print_LG = &empty_output_hook;
		hook_print_SD = &empty_output_hook;
		hook_print_TR = &empty_output_hook;
	}
	else
	{
		if ( clopts.selec_state_and_error_SE )
			hook_print_SE = &print_pathnerr;
		else
			hook_print_SE = &empty_output_hook;

		if ( clopts.selec_lagrange_grad_LG )
			hook_print_LG = &print_LG;
		else
			hook_print_LG = &empty_output_hook;

		if ( clopts.selec_SD_converged_SD )
			hook_print_SD = &print_SD;
		else
			hook_print_SD = &empty_output_hook;

		if ( clopts.selec_target_reached_TR )
			hook_print_TR = &print_TR;
		else
			hook_print_TR = &empty_output_hook;
	}
	if ( !clopts.selec_sim )
	{
		vme.tcp_connect();
		vme.update_poshead( qu, C );
	}

	/*
	 * Enter the loop which will take us through all waypoints.
	 */
	while ( current_tgt_no < C.ntgt )
	{
		time_to_tgt[current_tgt_no] = -wall_time();
		C.cur_tgt = &C.tgt[current_tgt_no * 2];
		nmpc.tgtdist = C.tgttol + 1;

		while ( nmpc.tgtdist > .1 )
		{
			C.horizon_loop += 1;
			sd_loop_time = -wall_time();

			/*   SD loop.  */
			opt.teeup();
			do
			{
				/*
				 * The core of the gradient decent is in the next few lines:
				 */
				nmpc.predict_horizon( qu, p, C );
				nmpc.set_tracking_errors( qu, p, C );
				nmpc.get_gradient( qu, p, C, opt.*thegrad );

			}
			while ( opt.iterate( qu, p, C ) );

			// This changes nothing in the calculation, but makes the endpoint
			// penalty rather visible in the plot_output.py animation.
			hook_print_SE( qu, p, C );
			hook_print_LG( qu, p, C, opt.*thegrad );
			sd_loop_time += wall_time();
			//hook_print_SD( &sd_loop, &sd_loop_time );
			C.control_step += C.C;

			vme.exec_control_horiz_dummy( qu, C );

			trip_cost += nmpc.stage_cost_execp( qu, p, C );

			for ( k = 0; k < C.N - C.C - 1; ++k )
			{
				qu[k].v = qu[k + C.C].v;
				qu[k].Dth = qu[k + C.C].Dth;
			}

			if ( C.control_step > MAX_NMPC_ITER * C.C )
			{
				sprintf( errnote,
				         "Reached %d NMPC steps without reaching tgt. Stopping.",
				         MAX_NMPC_ITER );
				report_error( TRAPPED_IN_NMPC_LOOP, errnote );
			}
		}
		time_to_tgt[current_tgt_no] += wall_time();
		hook_print_TR( &current_tgt_no, &time_to_tgt[current_tgt_no] );
		++current_tgt_no;
		printf( "# Accumulated cost of executed path: %f\n", trip_cost );
	}

	return 0;
}
