/*
 * nmpc-funcs.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-27
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

#include <cmath>
#include <unistd.h>
#include <stdlib.h>

#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"
#include "struct_wall.h"
#include "class_robot.h"

// from time-sync.cpp
double wall_time();

#define SEC_TO_USEC 1e-6

float* dist_from_wall(qnu& qu, wall& W)
{

	static float* A = (float*) malloc(2 * sizeof(float));
	float WPx = qu.x - W.x0;
	float WPy = qu.y - W.y0;
	float vx = W.x1 - W.x0;
	float vy = W.y1 - W.y0;
	float c1 = (vx * WPx + vy * WPy);
	if ( c1 <= 0)
	{
		A[0] = -WPx;
		A[1] = -WPy;
		return A;
	}
	float c2 = (vx*vx + vy*vy);
	if (c2 <= c1)
	{
		A[0] = W.x1 - qu.x;
		A[1] = W.y1 - qu.y;
		return A;
	}

	float b = c1/c2;
	A[0] = (W.x0 + b * vx) - qu.x;
	A[1] = (W.y0 + b * vy) - qu.y;
	return A;
}

/*
 * Initialization of dependent variables based on user input.
 */
void init_qu_and_p(qnu* qu, Lagr* p, nmpc& C)
{
	qu[0].x = +0.0;
	qu[0].Dx = C.cruising_speed
	           * cos(atan2((double) C.cur_tgt[1], (double) C.cur_tgt[0]));
	qu[0].y = +0.0;
	qu[0].Dy = C.cruising_speed
	           * sin(atan2((double) C.cur_tgt[1], (double) C.cur_tgt[0]));
	qu[0].th = atan2((double) C.cur_tgt[1], (double) C.cur_tgt[0]);
	p[0].sintk = sin(qu[0].th);
	p[0].costk = cos(qu[0].th);
	p[C.N - 1].p2 = +0.0;
	p[C.N - 1].p4 = +0.0;
	p[C.N - 1].p5 = +0.0;
	C.grang = 20 * M_PI;
	C.horizon_loop = 0;
	for (int k = 0; k < C.N; ++k)
	{
		qu[k].v = C.cruising_speed;
		qu[k].Dth = +0;
	}
}

/*!
 * This function evaluates the cost functional of a given path.
 */
float costfun(const qnu *qu, const Lagr *p, const nmpc &C)
{
	float J; // The cost
	J = C.Q0[0] * p[C.N - 1].ex * p[C.N - 1].ex
	    + C.Q0[1] * p[C.N - 1].ey * p[C.N - 1].ey;
	for (unsigned int k = 0; k < C.N - 1; k++)
	{
		J += 2 * p[C.N - 1].ex * p[C.N - 1].ex * C.Q0[0]
		     + 2 * p[C.N - 1].ey * p[C.N - 1].ey * C.Q0[1];
		for (unsigned int j = 0; j < C.nobst; j++)
			J += 1
			     / ((C.cur_tgt[0] - qu[0].x) * (C.cur_tgt[0] - qu[0].x)
			        + (C.cur_tgt[1] - qu[0].y)
			        * (C.cur_tgt[1] - qu[0].y) + C.eps);
	}
	return -J;
}

/*!
 * This function approximates the state trajectory of the system based on a set
 * of control input. It is used to optimize the control input by giving an
 * approximate path to build an approximation of the cost measure of possible
 * movements.
 *
 * The function returns the distance between the current robot position and the
 * current target.
 */
float predict_horizon(qnu* qu, Lagr* p, const nmpc &C)
{
	float dirx = C.cur_tgt[0] - qu[0].x;
	float diry = C.cur_tgt[1] - qu[0].y;
	// TODO: Store dist this to use in loop terminator.
	float dist = sqrt(dirx * dirx + diry * diry);
	dirx /= dist;
	diry /= dist;
	/*
	 * For all times in the horizon, provide an approximation of the state
	 * vector, based on a simple Euler integration scheme.
	 */
	for (unsigned int k = 1; k < C.N; k++)
	{
		qu[k].th = qu[k - 1].th + qu[k - 1].Dth * C.T; // Steering angle.
		p[k].costk = cos(qu[k].th);
		p[k].sintk = sin(qu[k].th);
		qu[k].x = qu[k - 1].x + qu[k - 1].Dx * C.T;     // X-position.
		qu[k].Dx = C.cruising_speed * p[k].costk;       // X-speed.
		qu[k].y = qu[k - 1].y + qu[k - 1].Dy * C.T;     // Y-position.
		qu[k].Dy = C.cruising_speed * p[k].sintk;       // Y-speed.
		p[k].ex = qu[k].x - (qu[0].x + C.cruising_speed * dirx * k * C.T);
		p[k].ey = qu[k].y - (qu[0].y + C.cruising_speed * diry * k * C.T);
	}
	return dist;
}

/*!
 * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange multipliers
 * are computed. The norm of the gradient vector is also sotred in the N+1th
 * element of the gradient array.
 */
void get_gradient(qnu* qu, Lagr* p, nmpc &C, float *grad)
{
	float* dW;

	int k;
	unsigned int j;
	float PhiX, PhiY, denom, difx, dify, gDth;

	grad[C.N] = 0.;

	p[C.N - 1].p1 = C.Q0[0] * p[C.N - 1].ex;
	p[C.N - 1].p3 = C.Q0[1] * p[C.N - 1].ey;
	qu[C.N - 2].Dth -= C.dg * C.R[0] * qu[C.N - 2].Dth;

	/*!
	 * Get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
	 * through each k in N. This involves computing the obstacle potential
	 * and Lagrange multipliers. Then, the control plan is updated by
	 * stepping against the direction of the gradient.
	 */
	for (k = C.N - 2; k >= 0; --k)
	{
		PhiX = 0.;
		PhiY = 0.;
		/*
		 * Compute the obstacle potential by looping through the list of
		 * obstacles:
		 */
		for (j = 0; j < C.nobst; j++)
		{
			difx = qu[k].x - C.obst[j * 2];
			dify = qu[k].y - C.obst[(j * 2) + 1];
			denom = difx * difx + dify * dify + C.eps;
			denom *= denom;
			PhiX += 2 * difx / denom;
			PhiY += 2 * dify / denom;
		}
		// Add the potential due to walls:
		for (j = 0; j < C.nwalls; ++j)
		{
			// Put the distances from the wall in here.
			dW = dist_from_wall(qu[k], C.walls[j]);
			denom = dW[0] * dW[0] + dW[1] * dW[1] + C.eps;
			denom *= denom;
			PhiX -= 2 * dW[0] / denom;
			PhiY -= 2 * dW[1] / denom;
		}
		// Compute the Lagrange multipliers:
		p[k].p1 = C.Q[0] * p[k].ex - PhiX + p[k + 1].p1;
		p[k].p2 = p[k + 1].p1 * C.T;
		p[k].p3 = C.Q[1] * p[k].ey - PhiY + p[k + 1].p3;
		p[k].p4 = p[k + 1].p3 * C.T;
		p[k].p5 = p[k + 1].p5 + p[k + 1].p4 * qu[k].v * p[k].costk
		          - p[k + 1].p2 * qu[k].v * p[k].sintk;
		grad[k] = C.R[0] * qu[k].Dth + p[k + 1].p5 * C.T;
		grad[C.N] += grad[k] * grad[k];
	}
	grad[C.N] = sqrt(grad[C.N]);
}

/*!
 * Swap float pointers A and B so that new A points to old B and vice versa.
 */
void swap_fptr(float** A, float** B)
{
	float* tmp;
	tmp = *A;
	*A = *B;
	*B = tmp;
}

/*!
 * Shifts the data by the control horizon.
 */
double exec_control_horiz_dummy(qnu* qu, const nmpc& C, robot* vme)
{
	double now = wall_time();
	/*
	 * For Diagnostics, just shift
	 */
	qu[0].x = qu[C.C].x;
	qu[0].Dx = qu[C.C].Dx;
	qu[0].y = qu[C.C].y;
	qu[0].Dy = qu[C.C].Dy;
	qu[0].th = qu[C.C].th;
	return now;
}

double exec_control_horiz_vme(qnu* qu, const nmpc& C, robot* vme)
{
	static double time_last_cmd_sent;
	double now;
	int k;

	for (k = 0; k < C.C; ++k)
	{
		now = wall_time();
		while ((now - time_last_cmd_sent) * .8 < C.T)
		{
			usleep((now - time_last_cmd_sent) * SEC_TO_USEC / 2);
			now = wall_time();
		}
		vme->Nav2_v(&qu[k].v, &qu[k].Dth);
		fprintf(stderr, "# DT: %18.15f", now - time_last_cmd_sent);
		time_last_cmd_sent = wall_time();
		vme->update_poshead(qu, C);
	}
	return now;
}
