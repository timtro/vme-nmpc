#!/usr/bin/env python
"""
 * plot_output.py
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

This code shows an animation of the path generation, illustrating not only the
motion through the field, but a comparison of the SD convergence results to the
reference line which the robot should with to track.

This code is also intended as a general demonstration of the usage of the
nmpc_stats_and_quantities and nmpc_output_parse modules.

"""

from argparse import ArgumentParser
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from modules.Nav2Robot import Nav2Robot
import matplotlib.gridspec as gridspec
import re


def init():
    history.set_data([], [])
    curpos.set_data([], [])


def update_plot(data):

    #Poll and unpack.
    X = vme.locationa()

    if np.sqrt(np.sum((X[:2] - update_plot.Xprv[:2])**2)) > .001:
        update_plot.executed_path[0].append(X[0])
        update_plot.executed_path[1].append(X[1])
        update_plot.executed_path[2].append(X[2])
        if len(update_plot.executed_path[0]) > 500:
            for i in update_plot.executed_path[:]:
                del i[0]
        update_plot.Xprv = X

    #TODO: Modify to store a trail of, say, 1000 data points in history.
    history.set_data(update_plot.executed_path[0], update_plot.executed_path[1])
    curpos.set_data(X[0], X[1])
    curpos.set_marker((3, 0, X[2] - 90))

    return history, curpos


parser = ArgumentParser(description='Plots the position and path history of a Nav2 machine.')
parser.add_argument('-o',
                    '--host',
                    dest='hostname',
                    default='localhost',
                    help='Address of turtle server.',
                    metavar='HOST')
parser.add_argument('-p',
                    '--port',
                    dest='hostport',
                    type=int,
                    default=5010,
                    help='Turtle is listening on this port.',
                    metavar='PORT')
parser.add_argument('-xr',
                    dest='xr',
                    default="[-5,5]",
                    help='x-axis range (two comma separated floats in brackets. Default: "[-5,5]").',
                    metavar='RANGE')
parser.add_argument('-yr',
                    dest='yr',
                    default="[-5,5]",
                    help='y-axis range (two comma separated floats in brackets. Default: "[-5,5]").',
                    metavar='RANGE')
parser.add_argument('-i', dest='ival', type=float, default=5, help='Animation refresh interval', metavar='IVAL')

args = parser.parse_args()

parseRange = re.compile('\s*\\[\\s*(-?\\d+)\\s*[,:]\\s*(-?\\d+)\\s*\\]\s*')

xr = [int(i) for i in parseRange.findall(args.xr)[0]]
yr = [int(i) for i in parseRange.findall(args.yr)[0]]

# Initialize the TCP/IP socket
print("Initializing connection to VirualME...")
vme = Nav2Robot(args.hostname, args.hostport)
vme.connect()
print("  done.")

update_plot.executed_path = [[], [], []]
update_plot.Xprv = vme.locationa()

fig = plt.figure()
# gs = gridspec.GridSpec(1, 1, width_ratios=1)
# ax = plt.axes([xr[0], xr[1], yr[0], yr[1]])

ax = fig.add_subplot(111, adjustable='box', aspect=1.0)
ax.set_xlim(xr[0], xr[1])
ax.set_ylim(yr[0], yr[1])
ax.grid()

ax.grid()

history, = ax.plot([], [], 'r-', lw=2)
curpos, = ax.plot([], [], 'r^', ms=15)

# plt.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)
anim = animation.FuncAnimation(fig, update_plot, init_func=init, interval=args.ival, blit=False)
plt.show()
