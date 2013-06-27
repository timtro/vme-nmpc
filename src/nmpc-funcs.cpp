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

#include "struct_nmpc.h"
#include "struct_qnu.h"
#include "struct_Lagr.h"

float costfun( const qnu *qu, const Lagr *p, const nmpc &C )
  {
    float J; // The cost
    J = C.Q0[0] * p[C.N - 1].ex * p[C.N - 1].ex
            + C.Q0[1] * p[C.N - 1].ey * p[C.N - 1].ey;
    for ( unsigned int k = 0; k < C.N - 1; k++ )
      {
        J += 2 * p[C.N - 1].ex * p[C.N - 1].ex * C.Q0[0]
                + 2 * p[C.N - 1].ey * p[C.N - 1].ey * C.Q0[1];
        for ( unsigned int j = 0; j < C.nobst; j++ )
          J += 1
                  / ( ( C.tgt[0] - qu[0].x ) * ( C.tgt[0] - qu[0].x )
                          + ( C.tgt[1] - qu[0].y ) * ( C.tgt[1] - qu[0].y )
                          + C.eps );
      }
    return -J;
  }

/*!
 * This function approximates the state trajectory of the system based
 * on a set of control input. It is used to optimize the control input
 * by giving an approximate path to build an approximation of the cost
 * measure of possible movements.
 */
void predict_horizon( qnu *qu, Lagr *p, const nmpc &C )
  {
    float dirx = C.tgt[0] - qu[0].x;
    float diry = C.tgt[1] - qu[0].y;
    // TODO: Store dist this to use in loop terminator.
    float dist = sqrt(dirx * dirx + diry * diry);
    dirx /= dist;
    diry /= dist;
    /*
     * For all times in the horizon, provide an approximation of the state
     * vector, based on a simple Euler integration scheme.
     */
    for ( unsigned int k = 1; k < C.N; k++ )
      {
        qu[k].th = qu[k - 1].th + qu[k - 1].Dth * C.T; // Steering angle.
        p[k].costk = cos(qu[k].th);
        p[k].sintk = sin(qu[k].th);
        qu[k].x = qu[k - 1].x + qu[k - 1].Dx * C.T;    // X-position.
        qu[k].Dx = C.cruising_speed * p[k].costk;       // X-speed.
        qu[k].y = qu[k - 1].y + qu[k - 1].Dy * C.T;    // Y-position.
        qu[k].Dy = C.cruising_speed * p[k].sintk;       // Y-speed.
        p[k].ex = qu[k].x - ( qu[0].x + C.cruising_speed * dirx * k * C.T );
        p[k].ey = qu[k].y - ( qu[0].y + C.cruising_speed * diry * k * C.T );
      }
  }

/*!
 * Calculate gradient from ∂J = ∑∂H/∂u ∂u, then step the control set.
 */
void get_gradient( qnu *qu, Lagr *p, nmpc &C, float *grad )
  {
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

    for ( k = C.N - 2; k >= 0; --k )
      {
        PhiX = 0.;
        PhiY = 0.;
        /*
         * Compute the obstacle potential by looping through the list of
         * obstacles:
         */
        for ( j = 0; j < C.nobst; j++ )
          {
            /*
             * FIXME: Probably best to remove the vectors from C and use
             * arrays. Vectors could be used temporarily to expand the
             * data from the input file to memory. Error checking for
             * the number of coordinates can be done there.
             */
            difx = qu[k].x - C.obst[j * 2];
            dify = qu[k].y - C.obst[ ( j * 2 ) + 1];
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
        grad[k] = C.R[0] * qu[k].Dth + p[k + 1].p5 * C.T;
        grad[C.N] += grad[k] * grad[k];
      }
    grad[C.N] = sqrt(grad[C.N]);
  }
