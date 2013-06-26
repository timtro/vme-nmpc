/*
 * vme-nmpc.cpp
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

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "vme-nmpc.h"
#include "qnu.h"

#define SEC_TO_USEC 1E+6

int main( int argc, char **argv )
  {
    unsigned int sd_loop = 0, k = 0;
    float tgtdist, grad_dot_grad, max_dg;
    float *this_grad, *last_grad;
    bool at_max_dg = false, is_unstable = false;
    robot vme(NULL, NULL);
    nmpc C;

    parse_command_line(argc, argv, &vme);
//  TODO: Error checking to be sure C.N is defined!
    parse_input_file(C, vme.conffile());

    qnu *qu = (qnu*) calloc(C.N, sizeof(qnu));
    Lagr *p = (Lagr*) calloc(C.N, sizeof(Lagr));
    float *grad = (float*) calloc(C.N + 1, sizeof(float));

    qu[0].x = +0.0;
    qu[0].Dx = C.cruising_speed
            * cos(atan2((double) (*C.tgt)[1], (double) (*C.tgt)[0]));
    qu[0].y = +0.0;
    qu[0].Dy = C.cruising_speed
            * sin(atan2((double) (*C.tgt)[1], (double) (*C.tgt)[0]));
    qu[0].th = atan2((double) (*C.tgt)[1], (double) (*C.tgt)[0]);
    p[0].sintk = sin(qu[0].th);
    p[0].costk = cos(qu[0].th);
    p[C.N - 1].p2 = +0.0;
    p[C.N - 1].p4 = +0.0;
    p[C.N - 1].p5 = +0.0;
    C.grang = 20 * M_PI;
    C.horizon_loop = 0;
    //C.dg *= 2;
    tgtdist = 100; // Some large(ish) number to get us into the loop.
    for ( k = 0; k < C.N; ++k )
      {
        qu[k].v = C.cruising_speed;
        qu[k].Dth = +0;
      }

    printf("\n  Starting nmpc-pf v0.9 (cpp)\n");
    printf("  Time and Date\n\n");

    printf("  Integration time step           (T) : %f\n", C.T);
    printf("  Size of state vector            (m) : %d\n", C.m);
    printf("  Size of control vector          (n) : %d\n", C.n);
    printf("  MPC Horizon size                (N) : %d\n", C.N);
    printf("  Control horizon size            (C) : %d\n", C.C);
    printf("  Default SD mixing              (dg) : %f\n", C.dg);
    printf("  Desired cruising speed              : %f\n", C.cruising_speed);
    printf("  Obstacle peak inverse maximum (eps) : %f\n", C.eps);
    printf("  Terminal cost weight diag.     (Q0) : ");
    for ( k = 0; k < 2; ++k )
      printf("%f ", C.Q0[k]);
    printf("\n");
    printf("  Tracking cost weight diag.      (Q) : ");
    for ( k = 0; k < 2; ++k )
      printf("%f ", C.Q[k]);
    printf("\n");
    printf("  Control input cost weight diag. (R) : ");
    for ( k = 0; k < C.n; ++k )
      printf("%f ", C.R[k]);
    printf("\n");
    printf("  State vector cost weight diag.  (S) : ");
    for ( k = 0; k < C.m; ++k )
      printf("%f ", C.S[k]);
    printf("\n");
    printf("  Origin defaults to (0  0)\n");
    printf("  Desired target list     :\n    x         y\n");
    // TODO: Change ->size() when I properly test and store the target list
    for ( k = 0; k < C.tgt->size() / 2; ++k )
      printf("    %f %f\n", (*C.tgt)[2 * k], (*C.tgt)[2 * k + 1]);
    printf("  Point obstacle list     :\n    x         y\n");
    // TODO: Change ->size() when I properly test and store the target list
    for ( k = 0; k < C.obst->size() / 2; ++k )
      printf("    %f %f\n", (*C.obst)[2 * k], (*C.obst)[2 * k + 1]);

    /*
     * Enter the loop which will take us through all waypoints.
     * TODO: Add hooks to insert waypoints.
     */
    while (tgtdist > .1)
      {
        C.horizon_loop += 1;
        sd_loop = 0;
        last_cost = 0;
        while (1)
          {
            at_max_dg = false;
            sd_loop += 1;
            printf("# !HL : %d\n# !SL : %d\n", C.horizon_loop, sd_loop);
            predict_horizon(qu, p, C);
            printf("#k         x         y        Dx        Dy        th");
            printf("         v       Dth        ex        ey\n");
            for ( k = 0; k < C.N; ++k )
              {
                printf("%2d% 10.4f% 10.4f% 10.4f% 10.4f", k, qu[k].x, qu[k].y,
                        qu[k].Dx, qu[k].Dy);
                printf("% 10.4f% 10.4f% 10.4f% 10.4f% 10.4f\n", qu[k].th,
                        qu[k].v, qu[k].Dth, p[k].ex, p[k].ey);
              }
            this_cost = costfun(qu, p, C);
            printf("# this_cost : %f, last_cost : %f\n", this_cost, last_cost);
            get_gradient(qu, p, C, grad);

            for ( k = 0; k < C.N - 1; ++k )
              qu[k].Dth -= C.dg * grad[k];

            printf("# DG : %f, at_max_dg: %d\n", C.dg, at_max_dg);
            last_cost = this_cost;
            if ( grad[C.N] < .1 ) break;
          }
        C.control_step += C.C;
        qu[0].x = qu[C.C].x;
        qu[0].Dx = qu[C.C].Dx;
        qu[0].y = qu[C.C].y;
        qu[0].Dy = qu[C.C].Dy;
        qu[0].th = qu[C.C].th;
        for ( k = 0; k < C.N - C.C - 1; ++k )
          {
            qu[k].v = qu[k + C.C].v;
            qu[k].Dth = qu[k + C.C].Dth;
          }
        printf("# !HLC : %d\n", sd_loop);
      }
//    vme.tcp_connect();
//    vme.update_poshead();
//    usleep(2*SEC_TO_USEC);
    return 0;
  }
