#!/usr/bin/env python

from optparse import OptionParser
from sys import stdin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def grab_setup(infile, tgt, obs):
    line = infile.readline()
    while line[0] != '#':
        line = infile.readline()
        if ("Desired target list" in line):
            while 1:
                line = infile.readline()
                if is_number(line.split()[0]):
                    tgt.append([float(line.split()[0]), float(line.split()[1])]);
                else:
                    break
        if ("obstacle list" in line):
            while 1:
                line = infile.readline()
                if is_number(line.split()[0]):
                    obs.append([float(line.split()[0]), float(line.split()[1])]);
                else:
                    break

def grab_data():
    infile = grab_data.infile
    X, Y, Xr, Yr = [], [], [], []
    line = infile.readline()
    if len(line) == 0:
        return [], [], [], []
    while line[0] != '#':
        line = infile.readline()
        if len(line) == 0:
            return [], [], [], []
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return [], [], [], []
    while line[0] != '#':
            # k = int(line.split()[0])
            X.append(float(line.split()[1]))
            Y.append(float(line.split()[2]))
            Xr.append(X[len(X) - 1] - float(line.split()[8]))
            Yr.append(Y[len(Y) - 1] - float(line.split()[9]))
            line = infile.readline()
            if len(line) == 0:
                return [], [], [], []
    return X, Y, Xr, Yr

def animate(data):
    # update the data
    xdata, ydata, xrdata, yrdata = grab_data()
    # xmin, xmax = ax.get_xlim()
    lineA.set_data(xdata, ydata)
    lineB.set_data(xrdata, yrdata)
    return lineA, lineB

def init():
    lineA.set_data([], [])
    lineB.set_data([], [])
    return lineA, lineB

def Phi(x, y, obs):
    phi = 0
    for r in obs:
        phi = phi + 1 / ((r[0] - x) ** 2 + (r[1] - y) ** 2 + .08)
    return phi

parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
              help="Path to input file (output from vme-nmpc).", metavar="FILE")
parser.add_option("-i", "--interval", dest="interval",
              help="Graph refresh interval",)
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")
parser.add_option("-p", "--plain",
              action="store_true", dest="plaintrue", default=False,
              help="Don't look for obstacle and target information in the stream")
(options, args) = parser.parse_args()

if options.stdintrue :
    instream = stdin
    grab_data.infile = stdin
else:
    instream = open(options.filename, 'r')
    grab_data.infile = instream

tgt = []
obs = []
grab_setup(instream, tgt, obs)
print(tgt)
print("\n")
print(obs)

xr = [-.5, 10]
yr = [-2.5, 2.5]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_ylim(yr[0], yr[1])
ax.set_xlim(xr[0], xr[1])
if not options.plaintrue :
    x = np.linspace(xr[0], xr[1], 120)
    y = np.linspace(yr[0], yr[1], int(120 * (yr[1] - yr[0]) / (xr[1] - xr[0]))).reshape(-1, 1)
    ax.imshow(Phi(x, y, obs), cmap=plt.get_cmap('hot'), extent=[xr[0], xr[1], yr[0], yr[1]], origin='lower')
    ax.plot(*zip(*tgt), marker='o', ls='', color='r', markersize=15)
    # ax.plot(*zip(*obs), marker = 'o', ls = '')

lineA, = ax.plot([], [], 'ro-', lw=2)
lineB, = ax.plot([], [], 'yo-', lw=2)


ax.grid()
anim = animation.FuncAnimation(fig, animate, init_func=init, interval=float(options.interval), blit=True)
plt.show()
