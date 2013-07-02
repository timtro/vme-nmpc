#!/usr/bin/env python

from optparse import OptionParser
from sys import stdin
#import numpy as np
import pylab as pl

import nmpc_output_parse as pn


parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
              help="Path to input file (output from vme-nmpc).",
              metavar="FILE")
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")
(options, args) = parser.parse_args()

if options.stdintrue:
    instream = stdin
else:
    instream = open(options.filename, 'r')

nmpc = {}
sd_loops = []
sd_loop_time = []
path_and_error = []
lagrange_and_gradient = []

executed_path = [[], []]

if (pn.parse_welcome(instream, nmpc)):
    exit

while 1:
    path_and_error = []
    lagrange_and_gradient = []
    if (pn.parse_sd_stats(instream, sd_loops, sd_loop_time)):
        exit
    if (pn.parse_path_and_error(instream, nmpc, path_and_error)):
        break
    for k in range(0, nmpc["C"]):
        executed_path[0].append(path_and_error[k][0])
        executed_path[1].append(path_and_error[k][1])
    if (pn.parse_lagrangian_and_gradient(instream, nmpc,
        lagrange_and_gradient)):
        break

fig = pl.figure()
ax = fig.add_subplot(111)
#pl.rc('font',**{'family':'Nimbus Roman No9 L'})
ax.set_xlim(0, 10)
ax.set_ylim(-5, 5)
ax.grid()
ax.plot(executed_path[0], executed_path[1])
ax.plot(nmpc["obst"][0], nmpc["obst"][1], "ro")
ax.plot(nmpc["tgt"][0], nmpc["tgt"][1], "bo")

pl.show()
