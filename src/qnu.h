/*
 * qnu.h
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-04
 *
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

#ifndef __QNU_H__
#define __QNU_H__

/*!
 * The qnu structure holds q and u related variables for the robot. An array of
 * qnu holds the state and conrol information for the NMPC horizon.
 */
typedef struct qnutag {
  //! The x-coordinate.
  float x;
  //! The time rate-of-change of x.
  float Dx;
  //! The y-coordinate.
  float y;
  //! The time rate-of-change of y.
  float Dy;
  //! The angle from the x-axis of the direction of travel.
  float th;
  //! The steering rate. That is, the time rate-of-change of th.
  float Dth;
  //! The radial component of speed.
  float v;
} qnu;

/*!
 * The Lagr holds the langrange multipliers and related variables. An array of
 * these holds the lagrange multipliers for the NMPC horizon.
 */
typedef struct Lagrtag {
  //! The error of the x-coordinate.
  float ex;
  //! The error of the y-coordinate.
  float ey;
  /*!
   * The Lagrange multipliers used in the gradient decent are in p1-p5.
   */
  float p1;
  float p2;
  float p3;
  float p4;
  float p5;
  /*!
   * Because they are used a few times, these are stored rather than computed
   * Explicitly each time
   */
  float sintk;
  float costk;
} Lagr;

#endif /* __QNU_H_ */
