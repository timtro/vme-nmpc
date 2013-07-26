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

#ifndef __STRUCT_QNU_H__
#define __STRUCT_QNU_H__

/*!
 * The qnu structure holds q and u related variables for the robot. An array of
 * qnu holds the state and conrol information for the NMPC horizon.
 */
typedef struct qnu_tag
{
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

#endif // __STRUCT_QNU_H__
