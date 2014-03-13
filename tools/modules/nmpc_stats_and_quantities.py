"""
 * nmpc_stats_and_quantities.py
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

Description
===========

This module contains functions for computing stats and quantities on data
produced by the nmpc_output_parse module. Caution is required, however, as the
parsing module produces lists, and many functions here expect numpy arrays. In
these cases, simple recasting is all that is required (i.e., a = np.array(a)).
"""

import numpy as np
from itertools import izip

def minimum_distance_to_obstacle(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        min_dist_to_obst - The length of the smallest line that may be drawn
        between the (discrete) path and the obstacles in the environemtnt.
    """
    min_dist_to_obst = 0
    for ox,oy in izip(nmpc["obst"][:, 0], nmpc["obst"][:, 1]):
        dist = np.sqrt( (path[:,met['x']]-ox)**2
            + (path[:,met['y']]-oy)**2 ).min()
        if (dist < min_dist_to_obst) or (min_dist_to_obst == 0):
            min_dist_to_obst = dist
    return min_dist_to_obst


def minimum_turn_radius(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        min_turn_rad - What it says on the tin. Computed from v = R*omega.
    """
    min_turn_rad = 0
    for Dth, v in izip(path[:, met['Dth']], path[:, met['v']]):
        if Dth != 0:
            turn_rad = abs(v/Dth)
            if (min_turn_rad > turn_rad) or (min_turn_rad == 0):
                min_turn_rad = turn_rad
    return min_turn_rad

def path_length(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        path_length - integral of speed over time using trapezoids.
    """
    return np.sum( np.sqrt( (path[1:,met['x']]-path[:-1,met['x']])**2
        + (path[1:,met['y']] - path[:-1,met['y']])**2 ) )

def RMS_turn_rate(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        RMS_turn_rate - what it says on the tin.
    """
    return np.sqrt( np.average( path[:, met['Dth']]**2 ) )

def avg_speed(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        avg_speed - The average speed over path, using nmpc['T'] for the step.
    """
    return np.average(path[:, met['v']])

def obst_potential(nmpc, xr, yr):
    """
    Args:
        nmpc - as above
        xr - a triplet of the form [xmin, xmax, dx], the range of the x-axis.
        yr - the same as xr, for the y-axis.
    Returns:
        X - meshgrid array for x.
        Y - meshgrid array for y.
        Phi - the potential on the meshgrid created on the xy-plane.
    """
    X, Y = np.meshgrid(np.arange(xr[0], xr[1], xr[2]),
                       np.arange(yr[0], yr[1], yr[2]))
    Phi = np.zeros(X.shape)
    if ('obst' in nmpc):
        for x,y in izip(nmpc["obst"][:, 0], nmpc["obst"][:, 1]):
            Phi += 1 / ((x-X)**2 + (y-Y)**2 + nmpc["eps"])
    if ('walls' in nmpc):
        for x1,y1,x2,y2 in izip(
                nmpc["walls"][:,0],
                nmpc["walls"][:,1],
                nmpc["walls"][:,2],
                nmpc["walls"][:,3]
                ):
            # I have to find a more numpy-ier way to do this:
            for (i,j), val in np.ndenumerate(Phi):
                WPx, WPy = X[i,j] - x1, Y[i,j] - y1
                vx, vy = x2 - x1, y2 - y1
                c1, c2 = vx * WPx + vy * WPy, vx*vx + vy*vy
                if (c1 <= 0):
                    Phi[i,j] += 1/ ( (x1-X[i,j])**2
                        + (y1-Y[i,j])**2 + nmpc["eps"])
                elif (c2 <= c1):
                     Phi[i,j] += 1/ ( (x2-X[i,j])**2
                        + (y2-Y[i,j])**2 + nmpc["eps"])
                else:
                    dx, dy = x1+c1*vx/c2 - X[i,j], y1+c1*vy/c2 - Y[i,j]
                    Phi[i,j] += 1/ ( dx**2 + dy**2 + nmpc["eps"])
    return X, Y, Phi
