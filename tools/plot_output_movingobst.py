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

from optparse import OptionParser
from sys import stdin, exit
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.animation as animation

from modules import nmpc_output_parse as nop
from modules import nmpc_stats_and_quantities as nsq

def empty_func():
    return
def empty_func(x):
    return

def update_plot(data):
    """
    This function is the blood and guts of the program. It mines the information
    from the input stream and updates the plot data for each step of the
    animation.
    """
    # update the data
    update_plot.exec_path
    update_plot.filepos
    update_plot.pause
    update_plot.isstep
    if not update_plot.pause:
        while 1:
            path_and_error = []
            # Read the first line after the previous parse. This line should
            # contain the block label (SD), (SE), (LG) or (TR).
            if not options.stdintrue:
                curpos = instream.tell()
            line = instream.readline()
            if len(line) == 0:
                update_plot.pause = True
                return ax1xy, ax1xryr, execp, ax1mobst
            if '(SE)' in line:
                if not options.stdintrue:
                    update_plot.filepos.append(curpos)
                if len(se_meta) == 0:
                    if (nop.get_block_meta(instream, se_meta)):
                        update_plot.pause = True
                        return ax1xy, ax1xryr, execp, ax1mobst
                    del se_meta['k']
                    # I hate doing this, but for now, parse_path_and_error()
                    # doesn't store k, it needs to be taken from the metadata.
                    for key in se_meta:
                        se_meta[key] -= 1
                nop.parse_path_and_error(instream, nmpc, path_and_error)
                path_and_error = np.array(path_and_error)
                break

        # xmin, xmax = ax.get_xlim()
        xrdata = path_and_error[:, se_meta['x']] - path_and_error[:, se_meta['ex']]
        yrdata = path_and_error[:, se_meta['y']] - path_and_error[:, se_meta['ey']]
        ax1xy.set_data(path_and_error[:, se_meta['x']],
                       path_and_error[:, se_meta['y']])
        ax1xryr.set_data(xrdata, yrdata)
        nmpc['obst'][:,1] += 0.4 * nmpc['C'] * nmpc['T']
        ax1mobst.set_data(nmpc['obst'][:,0], nmpc['obst'][:,1])
        for k in range(0, nmpc['C']):
            update_plot.exec_path[0].append(path_and_error[k, se_meta['x']])
            update_plot.exec_path[1].append(path_and_error[k, se_meta['y']])
        execp.set_data(update_plot.exec_path[0], update_plot.exec_path[1])
    if update_plot.isstep:
        update_plot.isstep = False
        update_plot.pause = True
    return ax1xy, ax1xryr, execp, ax1mobst

def init():
    ax1xy.set_data([], [])
    ax1xryr.set_data([], [])
    execp.set_data([], [])
    return ax1xy, ax1xryr, execp

def on_key(event):
    """
    Handle key press events.
    """
    on_key.instream
    if (event.key == 'left'):
        if not options.stdintrue:
            instream.seek(update_plot.filepos[-50])
            del update_plot.filepos[-50:]
            del update_plot.exec_path[0][-nmpc['C'] * 50:]
            del update_plot.exec_path[1][-nmpc['C'] * 50:]
    elif (event.key == ' '):
        update_plot.pause ^= True
    elif (event.key == 'right' and update_plot.pause):
        update_plot.isstep = True
        update_plot.pause = False
    elif (event.key == 'q'):
        quit(0)
# Useful for you to find the name of a key to expand the bindings:
#    else:
#        print('You pressed', event.key, event.xdata, event.ydata)


################################################################################
#
# Begin by parsing the command line options:
parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
              help="Path to input file (output from vme-nmpc).", metavar="FILE")
parser.add_option("-i", "--interval", dest="interval",
              help="Graph refresh interval",)
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")
(options, args) = parser.parse_args()
# Deal with whatever commandline arugents are given for the input stream:
if options.stdintrue :
    instream = stdin
    update_plot.instream = stdin
else:
    instream = open(options.filename, 'r')
    update_plot.instream = instream
    on_key.instream = instream

# Initialize variables, arrays, lists and dicts.
options.interval = float(options.interval)
path_and_error = []
executed_path = []
se_meta = {}
nmpc = {}
update_plot.exec_path = [[], []]
update_plot.filepos = []
update_plot.pause = False
update_plot.isstep = False

# Parse the welcome message
if (nop.parse_welcome(instream, nmpc) == 1):
    exit(1)
# As suggested in the module headers.
nmpc['tgt'] = np.transpose(np.array(nmpc['tgt']))
if ('obst' in nmpc):
    nmpc['obst'] = np.transpose(np.array(nmpc['obst']))
if ('walls' in nmpc):
    nmpc['walls'] = np.array(nmpc['walls'])

x_features = [nmpc['tgt'][:, 0].max(), nmpc['tgt'][:, 0].min()]
y_features = [nmpc['tgt'][:, 1].max(), nmpc['tgt'][:, 1].min()]
if ('obst' in nmpc):
    x_features.extend([nmpc['obst'][:, 0].max(),
        nmpc['obst'][:, 0].min()])
    y_features.extend([nmpc['obst'][:, 1].max(),
        nmpc['obst'][:, 1].min()])
if ('walls' in nmpc):
    x_features.extend([nmpc['walls'][:,::2].max(),
        nmpc['walls'][:,::2].min()])
    y_features.extend([nmpc['walls'][:,1::2].max(),
        nmpc['walls'][:,1::2].min()])
xrmin, xrmax = (min(x_features), max(x_features))
yrmin, yrmax = (min(y_features), max(y_features))
xcent, ycent = (xrmin + (xrmax - xrmin)/2,
    yrmin + (yrmax - yrmin)/2)
xyr = int(1.5*max([(xrmax - xrmin), (yrmax - yrmin)])/2)
xr, yr = ([xcent - xyr, xcent + xyr],
    [ycent - xyr, ycent + xyr])


X, Y, Phi = nsq.obst_potential(nmpc, [xr[0], xr[1], .05], [yr[0], yr[1], .05])
Phiamax = np.amax(Phi)
fig = plt.figure(figsize=(11, 8.5), dpi=94, facecolor='#efefef')
gs = gridspec.GridSpec(1, 1)

# We don't want to respond to keyboard events if we are sreaming live.
if not options.stdintrue:
    cid = fig.canvas.mpl_connect('key_press_event', on_key)

# All of the axis 1 stuff:
ax1 = fig.add_subplot(gs[0], adjustable='box', aspect=1.0)
ax1.set_xlim(xr[0], xr[1])
ax1.set_ylim(yr[0], yr[1])
ax1xy, = ax1.plot([], [], 'ro-', lw=3, ms=3)
ax1xryr, = ax1.plot([], [], 'yo-', lw=4, ms=3)
ax1mobst, = ax1.plot([], [], 'ro', ms=10)
execp, = ax1.plot([], [], 'r-')
#ax1.contour(X, Y, Phi, alpha=.3, cmap='jet',
#    levels=np.arange(0, Phiamax, Phiamax/20))
ax1.imshow(Phi, cmap=plt.get_cmap('jet'),
    extent=[xr[0], xr[1], yr[0], yr[1]], origin='lower')
ax1.grid()

# Tighten the figure and begin the animation:
plt.tight_layout(pad=1.08, h_pad=None, w_pad=None, rect=None)
anim = animation.FuncAnimation(fig, update_plot, init_func=init,
                               interval=options.interval, blit=True)

#anim.save('/home/timtro/Desktop/test.mp4', bitrate=1400)
plt.show()