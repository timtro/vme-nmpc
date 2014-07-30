/*
 * class_SDmin.cpp
 * Author : Timothy A.V. Teatro
 * Date   : Apr 2014
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2014 - Timothy A.V. Teatro
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

#include "class_SDmin.h"
#include "inc_errhandler.h"

#include <cstdlib>

char errnote[256];

optimizer::optimizer( const qnu* qu, const Lagr* p, const nmpc& C, int max_sd_iters )
{
	grad = ( float* ) calloc( C.N + 1, sizeof( float ) );
	last_grad = ( float* ) calloc( C.N + 1, sizeof( float ) );
	dg = C.dg;
	tol = 0.1;
	sd_loop = 0;
	max_iter = max_sd_iters;
}

optimizer::~optimizer()
{
	free( grad );
	free( last_grad );
}

/*!
 * Swap float pointers A and B so that new A points to old B and vice versa.
 */
void optimizer::swap_fptr( float** A, float** B )
{
	float* tmp;
	tmp = *A;
	*A = *B;
	*B = tmp;
}

bool optimizer::iterate( qnu* qu, Lagr* p, nmpc& C )
{
	unsigned int k;
	float grad_dot_grad = 0.;

	sd_loop += 1;

	for ( k = 0; k < C.N; ++k )
	{
		grad_dot_grad += grad[k] * last_grad[k];
	}

	/*
	 * Detect direction changes in the gradient by inspecting the
	 * product <grad, last_grad>. If it is positive, then the
	 * iterations are successfully stepping to the minimum, and we
	 * can accelerate by increasing dg. If we overshoot (and the
	 * product becomes negative), then backstep and drop dg to a
	 * safe value.
	 */
	if ( grad_dot_grad > 0 )
	{
		dg *= 2;
		for ( k = 0; k < C.N; ++k )
		{
			qu[k].Dth -= dg * grad[k];
		}
	}
	else
	{
		dg = 0.1; // TODO: Adaptive.
		for ( k = 0; k < C.N; ++k )
		{
			qu[k].Dth += dg * grad[k];
		}
	}
	swap_fptr( &grad, &last_grad );
	if ( last_grad[C.N] < tol )
		return false;
	if ( sd_loop >= max_iter )
	{
		sprintf( errnote, "Reached %d SD iterations. Stopping.",
		         sd_loop );
		report_error( EXCEEDED_MAX_SD_ITER, errnote );
	}
	return true;
}

void optimizer::teeup()
{
	sd_loop = 0;
}