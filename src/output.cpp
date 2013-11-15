/*
 * output.cpp
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-28
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

#include <time.h>

#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"

void empty_output_hook(const qnu* qu, const Lagr* p, const nmpc& C)
{
}

void empty_output_hook(const qnu* qu, const Lagr* p, const nmpc& C,
                       const float* grad)
{
}

void empty_output_hook(const unsigned int* a, const double* b)
{
}

void print_greeting(const nmpc& C)
{
	unsigned int k;
	time_t startup_time;

	time(&startup_time);
	printf("\n Starting vme-nmpc v0.9-alpha\n");
	printf("  %s\n\n", ctime(&startup_time));

	printf("  Integration time step                   (T) : %f\n", C.T);
	printf("  Size of state vector                    (m) : %d\n", C.m);
	printf("  Size of control vector                  (n) : %d\n", C.n);
	printf("  MPC Horizon size                        (N) : %d\n", C.N);
	printf("  Control horizon size                    (C) : %d\n", C.C);
	printf("  Default SD mixing                      (dg) : %f\n", C.dg);
	printf("  Desired cruising speed     (cruising_speed) : %f\n",
	       C.cruising_speed);
	printf("  Obstacle peak inverse maximum         (eps) : %f\n", C.eps);
	printf("  Terminal cost weight diag.             (Q0) : ");
	for (k = 0; k < 2; ++k)
		printf("%f ", C.Q0[k]);
	printf("\n");
	printf("  Tracking cost weight diag.              (Q) : ");
	for (k = 0; k < 2; ++k)
		printf("%f ", C.Q[k]);
	printf("\n");
	printf("  Control input cost weight diag.         (R) : ");
	for (k = 0; k < C.n; ++k)
		printf("%f ", C.R[k]);
	printf("\n");
	printf("  State vector cost weight diag.          (S) : ");
	for (k = 0; k < C.m; ++k)
		printf("%f ", C.S[k]);
	printf("\n");
	printf("  Origin defaults to (0  0)\n");
	printf("  Desired target list (tgt) :\n");
	for (k = 0; k < C.ntgt; ++k)
		printf("    % f % f\n", C.tgt[2 * k], C.tgt[2 * k + 1]);
	printf("  Point obstacle list (obst) :\n");
	for (k = 0; k < C.nobst; ++k)
		printf("    % f % f\n", C.obst[2 * k], C.obst[2 * k + 1]);
	printf("  List of wall segments (walls) :\n");
	for (k = 0; k < C.nwalls; ++k)
		printf("    % f % f - % f % f\n", C.walls[k].x0, C.walls[k].y0,
		       C.walls[k].x1, C.walls[k].y1);
	printf("#\n");
}

void print_pathnerr(const qnu* qu, const Lagr* p, const nmpc& C)
{
	printf("# (SE) State and error information:\n");
	printf("#k         x         y        Dx        Dy        th");
	printf("         v       Dth        ex        ey\n");
	for (unsigned int k = 0; k < C.N; ++k)
	{
		printf("%2d % 10.4f % 10.4f % 10.4f % 10.4f", k, qu[k].x, qu[k].y,
		       qu[k].Dx, qu[k].Dy);
		printf(" % 10.4f % 10.4f % 10.4f % 10.4f % 10.4f\n", qu[k].th, qu[k].v,
		       qu[k].Dth, p[k].ex, p[k].ey);
	}
}

void print_LG(const qnu* qu, const Lagr* p, const nmpc& C, const float* grad)
{
	printf("# (LG) Lagrange multipliers and gradient:\n");
	printf("#k        p1        p2        p3        p4        p5");
	printf("   grad[k]\n");
	for (unsigned int k = 0; k < C.N; ++k)
	{
		printf("%2d % 10.4f % 10.4f % 10.4f % 10.4f % 10.4f % 10.4f\n", k,
		       p[k].p1, p[k].p2, p[k].p3, p[k].p4, p[k].p5, grad[k]);
	}
}

void print_SD(const unsigned int* sd_loop, const double* sd_loop_time)
{
	printf("# (SD) loop completed.\n#sd_loop sd_loop_time\n");
	printf("%8d % 14.9f\n", *sd_loop, *sd_loop_time);
}

void print_TR(const unsigned int* current_tgt_no, const double* time_to_tgt)
{
	printf("# (TR) Target reached.\n#current_tgt_no time_to_tgt\n");
	printf("%15d %11f\n", *current_tgt_no, *time_to_tgt);
}
