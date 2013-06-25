/*
 * nmpc.h
 * This file is part of vme-nmpc
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

#ifndef __NMPC_H__
#define __NMPC_H__

#include <vector>

typedef struct nmpc_tag {

  float T;                   // Sampling time step (s)
  unsigned int N;            // Prediction Horizon
  unsigned int C;            // Control Horizon
  unsigned int m;            // State space dimension
  unsigned int n;            // Control space dimension
  unsigned int control_step; // Counter for control steps taken
  unsigned int horizon_loop; // Counter for Horizon loops completed
  /*
   * dg is the gradient mixing factor which controls the rate of decent to
   * the optimal control horizon. This factor affects the stability of
   * convergence, so it must be set with caution. If heuristic convergence
   * tracking is implimented, then this value will be adjusted
   * automatically during run time, and the initialization value is just a
   * safe starting value.
   */
  float dg;
  // The cruising speed is the desired safe speed for the robot to
  // follow a line. It should be fast enough to get quickly through
  // the rooms, but not so fast that the robot will topple if it must
  // make quick course corrections.
  float cruising_speed;
  // tgt is a 2D column vector to the desired target.
  std::vector<float> *tgt;
  // obst is a matrix containing a concainated list of 2D column
  // vectors of point obstacle coordinates.
  std::vector<float> *obst;
  // Q0 is the weighting matrix associated with the tracking error in
  // the last state in the prediction horizon.
  float *Q0;
  // Q is the weighting matrix associated with the cost term for the
  // tracking error
  float *Q;
  // S is the weighting matrix associated with the cost term for the
  // state vector.
  float *S;
  // R is the weighting matrix associated with the cost term of the
  // control input vector.
  float *R;
  // eps is the epsilon costant in the point obstacle potential.
  float eps;
  /*
   * grang is the direction of the gradient. This is tedted to determine
   * the decent rate
   */
  float grang;
} nmpc;

#endif
