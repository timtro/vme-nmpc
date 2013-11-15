#!/usr/bin/env python
"""
 * analyse_output.py
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

This code creates a summary of the executed path described in the vme-nmpc
output.

This code is also intended as a general demonstration of the usage of the
nmpc_stats_and_quantities and nmpc_output_parse modules.

"""

from optparse import OptionParser
from sys import stdin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import csv
import os

from modules import nmpc_output_parse as nop
from modules import nmpc_stats_and_quantities as nsq

#
# Parse command line options.
#
parser = OptionParser()
parser.add_option("-f", "--file", dest="filename", default='',
              help="Path to input file (output from vme-nmpc).",
              metavar="FILE")
parser.add_option("-c", "--csv", dest="csvout", default='',
              help="Path to CSV output file.",
              metavar="FILE")
parser.add_option("-p", "--pdf", dest="pdfout", default='',
              help="Path to PDF output file.",
              metavar="FILE")
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")

(options, args) = parser.parse_args()
if options.stdintrue:
    instream = stdin
else:
    instream = open(options.filename, 'r')

# Initialize lists and dicts.
nmpc = {}
sd_loops = []
sd_loop_time = []
path_and_error = []
lagrange_and_gradient = []
tgt_time = []
executed_path = []
sd_meta = {}
se_meta = {}
lg_meta = {}
tr_meta = {}
sd_fail = False
loop_trap = False

# Parse the welcome message
if (nop.parse_welcome(instream, nmpc) == 1):
    exit(1)
#
# Start parsing the bulk of the output.
#
# Loop until we hit the end of the data file. This loop identifies the output
# headers, records the column meta-data and then passes the stream pointer to
# functions which record the specific output blocks.
#
# The blocks are labelled SD, SE, LG and TR. SD marks the end of the gradient
# decent loop and the corresponding output block lists the number of loops and
# wall time of the gradient decent convergence. SE is the state and error,
# including control elements. LG has info about Lagrange multipliers and
# the gradient and TR marks where the target is reached and the wall-time for
# that waypoint is listed.
#

while 1:
    path_and_error = []
    lagrange_and_gradient = []
    # Read the first line after the previous parse. This line should contain
    # the block label (SD), (SE), (LG) or (TR).
    line = instream.readline()
    if len(line) == 0:
        break
    # Detect the report of SD convergence.
    if '(SD)' in line:
        # If we don't have the meta-data, store it.
        if len(sd_meta) == 0:
            if (nop.get_block_meta(instream, sd_meta)):
                break
        if (nop.parse_sd_stats(instream, sd_loops, sd_loop_time)):
            break
    # Detect the state and error output block (SE)
    if '(SE)' in line:
        if len(se_meta) == 0:
            if (nop.get_block_meta(instream, se_meta)):
                break
            del se_meta['k']
            # I hate doing this, but for now, parse_path_and_error() doesn't
            # store k, so it needs to be taken from the meta data.
            for key in se_meta:
                se_meta[key] -= 1
        if (nop.parse_path_and_error(instream, nmpc, path_and_error)):
            break
        # Pull first C x,y,v,Dth into the executed path.
        for k in range(0, nmpc["C"]):
            executed_path.append(path_and_error[k])
    # Detect the (LG) Lagrangian and Gradient output block
    if '(LG)' in line:
        if len(lg_meta) == 0:
            if (nop.get_block_meta(instream, lg_meta)):
                break
            del lg_meta['k']
            # I hate doing this, but for now, parse_lagrangian_and_gradient()
            # doesn't store k, so it needs to be taken from the meta data.
            for key in lg_meta:
                lg_meta[key] -= 1
        if (nop.parse_lagrangian_and_gradient(instream, nmpc,
            lagrange_and_gradient)):
            break
    # Detect the target-reached (TR) output block.
    if '(TR)' in line:
        if len(tr_meta) == 0:
            if (nop.get_block_meta(instream, tr_meta)):
                break
        if (nop.parse_tgt_wall_time(instream, tgt_time)):
            break
    if '0x20' in line:
        sd_fail = True
        exit(0)
        break
    if '0x21' in line:
        loop_trap = True
        exit(0)
        break

executed_path = np.array(executed_path)
tgt_time = np.array(tgt_time)
nmpc["obst"] = np.transpose(np.array(nmpc["obst"]))
nmpc["tgt"] = np.transpose(np.array(nmpc["tgt"]))
nmpc["walls"] = np.array(nmpc["walls"])

if (len(se_meta) != 0):
    min_dist_to_obst = nsq.minimum_distance_to_obstacle(nmpc, executed_path,
                                                        se_meta)
    min_turn_radius = nsq.minimum_turn_radius(nmpc, executed_path, se_meta)
    path_length = nsq.path_length(nmpc, executed_path, se_meta)
    RMS_turn_rate = nsq.RMS_turn_rate(nmpc, executed_path, se_meta)
    avg_speed = nsq.avg_speed(nmpc, executed_path, se_meta)
else:
    avg_speed = float('nan')
    path_length = float('nan')
    RMS_turn_rate = float('nan')
    min_turn_radius = float('nan')
    min_dist_to_obst = float('nan')

total_path_time = nmpc["T"] * executed_path.shape[0]

if tr_meta != {}:
    total_wall_time = sum(tgt_time[:,tr_meta['time_to_tgt']])
else:
    total_wall_time = float('NaN')

xrmin = min([min(nmpc['obst'][:, 0]), min(nmpc['tgt'][:, 0]), 0., nmpc['walls'][:,::2].min()])
xrmax = max([max(nmpc['obst'][:, 0]), max(nmpc['tgt'][:, 0]), 0., nmpc['walls'][:,::2].max()])
yrmin = min([min(nmpc['obst'][:, 1]), min(nmpc['tgt'][:, 1]), 0., nmpc['walls'][:,1::2].min()])
yrmax = max([max(nmpc['obst'][:, 1]), max(nmpc['tgt'][:, 1]), 0., nmpc['walls'][:,1::2].max()])
xcent = xrmin + (xrmax - xrmin)/2
ycent = yrmin + (yrmax - yrmin)/2
xyr = int(1.5*max([(xrmax - xrmin), (yrmax - yrmin)])/2)
xr = [xcent - xyr, xcent + xyr]
yr = [ycent - xyr, ycent + xyr]

#
# We've parsed the file, and computed the core statistics. Now create the
# report document.
#
plt.rc('font', **{'family': 'sans', 'size': 12})

fig = plt.figure(figsize=(11, 8.5), dpi=94, facecolor='#efefef', edgecolor='k')
gs = gridspec.GridSpec(4, 2, width_ratios=[1, 1.5], height_ratios=[1, 1, 1, 1])
ax1 = plt.subplot(gs[0:2, 0], axisbg="#ffffff", adjustable='box', aspect=1.0)
ax2 = plt.subplot(gs[0, -1], axisbg="#ffffff", adjustable='box')
ax3 = plt.subplot(gs[1, -1], axisbg="#ffffff", adjustable='box')
ax4 = plt.subplot(gs[2, -1], axisbg="#ffffff", adjustable='box')
plt.yticks(np.arange(.3, .5, .05))
ax5 = plt.subplot(gs[3, -1], axisbg="#ffffff", adjustable='box')
ax3.ticklabel_format(style='sci', axis='y', scilimits=(0, 0))
plt.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)

plt.figtext(.02,.02, 'Total path length [m]        : ' \
                + '{:< 8.4f}'.format(path_length)
            + "\nTotal path duration [s]      : " \
                + '{:< 8.4f}'.format(total_path_time)
            + "\nMin distance to obstacle [m] : " \
                + '{:< 8.4f}'.format(min_dist_to_obst)
            + "\nMaximum turn rate [rad/s]    : " \
                + '{:< 8.4f}'.format(max(executed_path[3]))
            + "\nSmallest turn radius [m]     : " \
                + '{:< 8.4f}'.format(min_turn_radius)
            + "\nAverage speed [m/s]          : " \
                + '{:< 8.4f}'.format(avg_speed)
            + "\nRMS turn rate [rad/s]        : " \
                + '{:< 8.7f}'.format(RMS_turn_rate)
            + "\nTotal trip wall time [s]     : " \
                + '{:< 8.7f}'.format(total_wall_time),
            family='monospace', verticalalignment='bottom')

ax1.text(.98, .98, 'Path [m, m]',
        horizontalalignment='right',
        verticalalignment='top',
        transform=ax1.transAxes)
ax1.set_xlim(xr[0], xr[1])
ax1.set_ylim(yr[0], yr[1])
ax1.grid()
X,Y,Phi = nsq.obst_potential(nmpc, [xr[0],xr[1],.1], [yr[0], yr[1], .1])
Phiamax = np.amax(Phi)
ax1.plot(executed_path[:, se_meta['x']], executed_path[:, se_meta['y']], 'k-')
ax1.plot(nmpc["obst"][:,0], nmpc["obst"][:,1], 'ko', ms=3)
ax1.plot(nmpc["tgt"][:,0], nmpc["tgt"][:,1], 'k^')
cp = ax1.contour(X, Y, Phi, alpha=.3, cmap='jet',
            levels=np.arange(0, Phiamax, Phiamax/20), lw=.75)

ax2.text(.98, .95, 'Convergence loop histogram [loops, freq]',
        horizontalalignment='right',
        verticalalignment='top',
        transform=ax2.transAxes)
ax2.hist(sd_loops, np.arange(1., max(sd_loops)+1., 1.) - .5, rwidth=.3,
         facecolor='k')
ax2.grid()

ax3.text(.98, .95, 'SD iterations v.s. wall-time [loops, s]',
        horizontalalignment='right',
        verticalalignment='top',
        transform=ax3.transAxes)
#loop_time_poly_fit = np.polyfit(sd_loops, sd_loop_time, 1)
#loop_time_poly = np.poly1d(loop_time_poly_fit)
avg_iter_time = np.sum(sd_loop_time) / np.sum(sd_loops)
ax3.text(.02, .95, '{:5.4g}'.format(avg_iter_time)
         + ' sec/iter',
        horizontalalignment='left',
        verticalalignment='top',
        transform=ax3.transAxes)
ax3.plot(sd_loops, sd_loop_time, 'o', color='k', ms=3)
#ax3.plot(range(min(sd_loops), max(sd_loops)+1),
#         loop_time_poly(range(min(sd_loops), max(sd_loops)+1)), 'k-', lw=.5)
ax3.grid()

ax4.text(.98, .95, 'Speed control value [s, m/s]',
        horizontalalignment='right',
        verticalalignment='top',
        transform=ax4.transAxes)
ax4.plot(np.arange(0, executed_path.shape[0]) * nmpc["T"],
         executed_path[:, se_meta['v']], color='k')
plt.setp(ax4.get_xticklabels(), visible=False)
ax4.grid()

ax5.text(.98, .95, 'Steering rate control value [s, rad/s]',
        horizontalalignment='right',
        verticalalignment='top',
        transform=ax5.transAxes)
ax5.plot(np.arange(0, executed_path.shape[0]) * nmpc["T"],
         executed_path[:, se_meta['Dth']], color='k')
ax5.grid()

if (options.pdfout == ''):
    plt.show()
else:
    plt.savefig(options.pdfout)

if (options.csvout != ''):
    csvexists = os.path.exists(
            os.path.join(os.path.dirname(os.path.realpath(__file__)),
                         options.csvout))
    with open(options.csvout, 'ab') as csvfd:
        csvw = csv.writer(csvfd, dialect='excel')

        dictmeta = ['Run ID', 'N', 'C', 'T [s]', "eps [m]", 'R', 'Q0', 'Q',
                    'Path Length [m]', 'Path Duration [s]',
                    'Min. Dist. to Obst. [m]', 'Max Turn Rate [rad/s]',
                    'Sm. Turn. Rad. [m]', 'Avg. Speed [m/s]',
                    'RMS Turn Rate [rad/s]', 'Wall Time [s]', 'SD fail?',
                    'Loop trap?']
        dictlist = [os.path.splitext(options.pdfout)[0], nmpc['N'], nmpc['C'],
                    nmpc['T'], nmpc['eps'], nmpc['R'], nmpc['Q0'][0],
                    nmpc['Q'][0], path_length, total_path_time,
                    min_dist_to_obst, max(executed_path[3]), min_turn_radius,
                    avg_speed, RMS_turn_rate, total_wall_time, sd_fail,
                    loop_trap]
#        if not csvexists:
#            csvw.writerow( dictmeta )
        csvw.writerow( dictlist )