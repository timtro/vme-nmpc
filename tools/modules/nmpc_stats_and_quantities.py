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
    for j in range(0, path.shape[0]):
        for k in range(0, nmpc["obst"].shape[0]):
            dx = nmpc["obst"][k, 0] - path[j, met['x']]
            dy = nmpc["obst"][k, 1] - path[j, met['y']]
            dist_to_obst = np.sqrt(dx ** 2 + dy ** 2)
            if (min_dist_to_obst > dist_to_obst) or (min_dist_to_obst == 0):
                min_dist_to_obst = dist_to_obst
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
    for k in range(0, path.shape[0]):
        if path[k, met['Dth']] != 0:
            turn_rad = abs(path[k, met['v']]
                           / path[k, met['Dth']])
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
    path_length = 0
    for v in path[:,met['v']]:
        path_length += v * nmpc['T']
    return path_length

def RMS_turn_rate(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        RMS_turn_rate - what it says on the tin.
    """
    RMS_turn_rate = 0
    for Dth in path[:, met['Dth']]:
        RMS_turn_rate = Dth ** 2
    RMS_turn_rate /= path.shape[0]
    RMS_turn_rate = np.sqrt(RMS_turn_rate)
    return RMS_turn_rate

def avg_speed(nmpc, path, met):
    """
    Args:
        nmpc - The output from nmpc_output_parse.parse_welcome.
        path - The state path with the pertinent data.
        met - The meta dictionary identifying the columns in path
    Returns:
        avg_speed - The average speed over path, using nmpc['T'] for the step.
    """
    avg_speed = 0
    for v in path[:, met['v']]:
        avg_speed += v
    avg_speed /= path.shape[0]
    return avg_speed

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
    for k in range(0, nmpc["obst"].shape[0]):
        Phi += 1/ ((nmpc["obst"][k, 0]-X)**2
                 + (nmpc["obst"][k, 1]-Y) **2 + nmpc["eps"])
    for k in range(0, nmpc["walls"].shape[0]):
        # I have to find a more numpy-ier way to do this:
        for i in range(0, Phi.shape[0]):
            for j in range(0, Phi.shape[1]):
                WPx = X[i,j] - nmpc["walls"][k,0]
                WPy = Y[i,j] - nmpc["walls"][k,1]
                vx = nmpc["walls"][k,2] - nmpc["walls"][k,0]
                vy = nmpc["walls"][k,3] - nmpc["walls"][k,1]
                c1 = (vx * WPx + vy * WPy)
                c2 = (vx*vx + vy*vy)
                if (c1 <= 0):
                    Phi[i,j] += 1/ ( (nmpc["walls"][k,0]-X[i,j])**2 +
                                (nmpc["walls"][k,1]-Y[i,j])**2 + nmpc["eps"])
                elif (c2 <= c1):
                     Phi[i,j] += 1/ ( (nmpc["walls"][k,2]-X[i,j])**2 +
                                (nmpc["walls"][k,3]-Y[i,j])**2 + nmpc["eps"])
                else:
                    dx = (nmpc["walls"][k,0]+c1*vx/c2) - X[i,j]
                    dy = (nmpc["walls"][k,1]+c1*vy/c2) - Y[i,j]
                    Phi[i,j] += 1/ ( dx**2 + dy**2 + nmpc["eps"])
    return X, Y, Phi
