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

#define MAX_SD_ITER 150
#define MAX_NMPC_ITER 4000

int main( int argc, char **argv )
  {

    void (*hook_output_path_and_error)( const qnu*,
            const Lagr*,
            const nmpc& ) = NULL;

    void (*hook_output_lagrange_grad)( const qnu*,
            const Lagr*,
            const nmpc&,
            const float* ) = NULL;

    char errnote[256];
    unsigned int sd_loop = 0, k = 0, current_tgt_no = 0;
    float tgtdist, grad_dot_grad = 0.;
    robot vme(NULL, NULL);

    double sd_loop_time;

    nmpc C;
    cl_opts clopts = { false };
    parse_command_line(argc, argv, &vme, &clopts);
    parse_input_file(C, vme.conffile());
    print_greeting(C);

    qnu *qu = (qnu*) calloc(C.N, sizeof(qnu));
    Lagr *p = (Lagr*) calloc(C.N, sizeof(Lagr));
    float *grad = (float*) calloc(C.N + 1, sizeof(float));
    float *last_grad = (float*) calloc(C.N + 1, sizeof(float));
    double *time_to_tgt = (double*) calloc(C.ntgt, sizeof(float));

    init_qu_and_p(qu, p, C);

    tgtdist = C.tgttol + 1; // Just to get us into the waypoint loop.
    C.horizon_loop = 0;

    if ( clopts.print_path_and_error )
      {
        hook_output_path_and_error = &print_pathnerr;
      }
    else
      {
        hook_output_path_and_error = &empty_output_hook;
      }
    if ( clopts.print_lagrange_grad )
      {
        hook_output_lagrange_grad = &print_lagrange_grad;
      }
    else
      {
        hook_output_lagrange_grad = &empty_output_hook;
      }

    /*
     * Enter the loop which will take us through all waypoints.
     */
    while (current_tgt_no < C.ntgt)
      {
        time_to_tgt[current_tgt_no] = -wall_time();
        C.cur_tgt = &C.tgt[current_tgt_no * 2];
        tgtdist = C.tgttol + 1;
        while (tgtdist > .1)
          {
            C.horizon_loop += 1;
            sd_loop = 0;
            sd_loop_time = -wall_time();
            while (1)
              {
                sd_loop += 1;
                grad_dot_grad = 0.;
                /*
                 * The core of the gradient decent is in the next few lines:
                 */
                tgtdist = predict_horizon(qu, p, C);
                get_gradient(qu, p, C, grad);
                for ( k = 0; k < C.N; k++ )
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
                    C.dg *= 2;
                    for ( k = 0; k < C.N; ++k )
                      {
                        qu[k].Dth -= C.dg * grad[k];
                      }
                  }
                else
                  {
                    C.dg = 0.1;
                    for ( k = 0; k < C.N; ++k )
                      {
                        qu[k].Dth += C.dg * grad[k];
                      }
                  }
                swap_fptr(&grad, &last_grad);
                if ( last_grad[C.N] < .1 ) break;
                if ( sd_loop >= MAX_SD_ITER )
                  {
                    sprintf(errnote, "Reached %d SD iterations. Stopping.",
                            sd_loop);
                    report_error(EXCEEDED_MAX_SD_ITER, errnote);
                  }
              }
            hook_output_path_and_error(qu, p, C);
            hook_output_lagrange_grad(qu, p, C, grad);
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
            sd_loop_time += wall_time();
            printf("# (SD) loop completed.\n#sd_loop sd_loop_time\n");
            printf("%8d % 14.9f\n", sd_loop, sd_loop_time);
            if ( C.control_step > MAX_NMPC_ITER * C.C )
              {
                sprintf(errnote,
                        "Reached %d NMPC steps without reaching tgt. Stopping.",
                        MAX_NMPC_ITER);
                report_error(TRAPPED_IN_NMPC_LOOP, errnote);
              }
          }
        time_to_tgt[current_tgt_no] += wall_time();
        printf("# (TR) Target reached.\n#current_tgt_no time_to_tgt\n");
        printf("%15d %11f\n", current_tgt_no, time_to_tgt[current_tgt_no]);
        ++current_tgt_no;
      }
//    vme.tcp_connect();
//    vme.update_poshead();
//    usleep(2*SEC_TO_USEC);
    return 0;
  }
