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

#ifndef __STRUCT_NMPC_H__
#define __STRUCT_NMPC_H__

#include "stdio.h"

typedef struct nmpc_tag {
  //! Sampling time step (s).
  float T;
  //! The NMPC prediction horizon length.
  unsigned int N;
  //! The NMPC control horizon.
  unsigned int C;
  //! State space dimensionality.
  unsigned int m;
  //! Control space dimensionality.
  unsigned int n;
  //! Counter for control steps taken.
  unsigned int control_step;
  //! Counter for Horizon loops completed.
  unsigned int horizon_loop;
  /*!
   * dg is the gradient mixing factor which controls the rate of decent to
   * the optimal control horizon. This factor affects the stability of
   * convergence, so it must be set with caution. If heuristic convergence
   * tracking is implimented, then this value will be adjusted
   * automatically during run time, and the initialization value is just a
   * safe starting value.
   */
  float dg;
  /*!
   *  The cruising speed is the desired safe speed for the robot to
   *  follow a line. It should be fast enough to get quickly through
   *  the rooms, but not so fast that the robot will topple if it must
   *  make quick course corrections.
   */
  float cruising_speed;

  /*!
   * Number of targets in target list
   */
  unsigned int ntgt;
  /*!
   *  tgt is a 2D column vector to the desired target.
   */
  float* tgt;
  /*!
   * Number of obstacles in obstacle list
   */
  unsigned int nobst;
  /*!
   *  obst is a matrix containing a concainated list of 2D column
   *  vectors of point obstacle coordinates.
   */
  float* obst;
  /*!
   *  Q0 is the weighting matrix associated with the tracking error in
   *  the last state in the prediction horizon.
   */
  float* Q0;
  /*!
   *  Q is the weighting matrix associated with the cost term for the
   *  tracking error
   */
  float* Q;
  /*!
   *  S is the weighting matrix associated with the cost term for the
   *  state vector.
   */
  float* S;
  /*!
   *  R is the weighting matrix associated with the cost term of the
   *  control input vector.
   */
  float* R;
  /*!
   *  eps is the epsilon costant in the point obstacle potential.
   */
  float eps;
  /*!
   * grang is the direction of the gradient. This is tedted to determine
   * the decent rate
   */
  float grang;
} nmpc;

#endif //__STRUCT_NMPC_H__
