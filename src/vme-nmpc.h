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

#include <cmath>

#include "robot.h"
#include "nmpc.h"
#include "qnu.h"

// from sockcomm.c
int init_vme_sock();

// from input.c
int parse_command_line( int, char **, robot * );
void parse_input_file( nmpc &, const char * );

/*
 * This function approximates the state trajectory of the system b  ased
 * on a set of control input. It is used to optimize the control input
 * by giving an approximate path to build an approximation of the cost
 * measure of possible movements.
 */
inline void predict_horizon( qnu *qu, Lagr *p, const nmpc &C )
  {
    float dirx = (*C.tgt)[0] - qu[0].x;
    float diry = (*C.tgt)[1] - qu[0].y;
    // TODO: Store dist this to use in loop terminator.
    float dist = sqrt(dirx * dirx + diry * diry);
    dirx /= dist;
    diry /= dist;
    /*
     * For all times in the horizon, provide an approximation of the state
     * vector, based on a simple Euler integration scheme.
     */
    for ( unsigned int k = 1; k < C.N; ++k )
      {
        qu[k].th = qu[k - 1].th + qu[k - 1].Dth * C.T; // Steering angle.
        p[k].costk = cos(qu[k].th);
        p[k].sintk = sin(qu[k].th);
        qu[k].x = qu[k - 1].x + qu[k - 1].Dx * C.T;    // X-position.
        qu[k].Dx = C.cruising_speed * p[k].costk;       // X-speed.
        qu[k].y = qu[k - 1].y + qu[k - 1].Dy * C.T;    // Y-position.
        qu[k].Dy = C.cruising_speed * p[k].sintk;       // Y-speed.
        p[k].ex = qu[k].x - (qu[0].x + C.cruising_speed * dirx * k * C.T);
        p[k].ey = qu[k].y - (qu[0].y + C.cruising_speed * diry * k * C.T);

      }
  }

/*
 * Calculate gradient from ∂J = ∑∂H/∂u ∂u, then step the control set.
 */
inline unsigned int gradient_step( qnu *qu, Lagr *p, nmpc &C )
  {
    int k;
    unsigned int j;
    float PhiX, PhiY, denom, difx, dify, gDth;
    float grad[19];

    float grad_norm = 0.;

    //TODO: Check p[C.N] -- Out of range?
    // NB: Canged to p[C.N-1] for now. Still check.
    p[C.N - 1].p1 = C.Q0[0] * p[C.N - 1].ex;
    p[C.N - 1].p3 = C.Q0[1] * p[C.N - 1].ey;
    qu[C.N - 2].Dth -= C.dg * C.R[0] * qu[C.N - 2].Dth;
    /*
     * To get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
     * through each k in N. This involves computing the obstacle potential
     * and Lagrange multipliers. Then, the control plan is updated by
     * stepping against the direction of the gradient.
     */
    printf(
            "#k     p1          p2        p3        p4        p5        PhiX      PhiY\n");
    printf("# 19  % 7.2f   % 7.2f   % 7.2f   % 7.2f   % 7.2f\n",
            p[C.N - 1].p1, p[C.N - 1].p2, p[C.N - 1].p3, p[C.N - 1].p4,
            p[C.N - 1].p5);
    for ( k = C.N - 2; k >= 0; --k )
      {
        PhiX = 0.;
        PhiY = 0.;
        /* Compute the obstacle potential by looping through the list of
         * obstacles:
         */
        for ( j = 0; j < C.obst->size() / 2; j++ )
          {
            /* FIXME: Probably best to remove the vectors from C and use
             * arrays. Vectors could be used temporarily to expand the
             * data from the input file to memory. Error checking for
             * the number of coordinates can be done there.
             */
            difx = qu[k].x - (*C.obst)[j * 2];
            dify = qu[k].y - (*C.obst)[(j * 2) + 1];
            denom = difx * difx + dify * dify + C.eps;
            denom *= denom;
            PhiX += 2 * difx / denom;
            PhiY += 2 * dify / denom;
          }
        // Compute the Lagrange multipliers:
        p[k].p1 = C.Q[0] * p[k].ex - PhiX + p[k + 1].p1;
        p[k].p2 = p[k + 1].p1 * C.T;
        p[k].p3 = C.Q[1] * p[k].ey - PhiY + p[k + 1].p3;
        p[k].p4 = p[k + 1].p3 * C.T;
        p[k].p5 = p[k + 1].p5 + p[k + 1].p4 * qu[k].v * p[k].costk
                - p[k + 1].p2 * qu[k].v * p[k].sintk;
        printf("# %2d  % 7.2f   % 7.2f   % 7.2f   % 7.2f   % 7.2f   % 7.2f   % 7.2f\n", k,
                p[k].p1, p[k].p2, p[k].p3, p[k].p4, p[k].p5, PhiX, PhiY);
      }
    for ( k = 0; k < C.N - 1; ++k )
      {
        // Compute gradient for each control element:
        gDth = C.R[0] * qu[k].Dth + p[k + 1].p5 * C.T;
        printf("# %f\n", gDth);
        /*
         * Add the gradients to the accumulator to later asses if we have
         * come close enough to the optimal solution:
         */
        grad_norm += gDth * gDth;
        // Now, step the control elements:
        qu[k].Dth = qu[k].Dth - C.dg * gDth;
      }
    /*
     * Out of the loop, we have an updated control plan. Check to see
     * if we are close enough to the ( or a) cost minimum by inspecting the
     * norm of the gradient accumulator:

     if (abs(atan2(gv, gDth) - C.grang) > M_PI_4/2)
     {
     C.dg /= 2;
     }
     else
     {
     C.dg *= 2;
     }
     C.grang = atan2(gv, gDth);
     */
    printf("# grad_norm: %f\n", sqrt(grad_norm));
    if ( sqrt(grad_norm) < 0.01 )
      {
        return 0;
      }
    else
      {
        return 1;
      }
  }
