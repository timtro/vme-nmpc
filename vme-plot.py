#! /usr/bin/python

from optparse import OptionParser
from sys import stdin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def grab_data():
    infile = grab_data.infile
    X,Y, Xr, Yr = [],[],[],[]
    line = infile.readline()
    if len(line) == 0:
        return [],[],[],[]
    while line[0] != '#':
        line = infile.readline()
        if len(line) == 0:
            return [],[],[],[]
    while line[0] == '#':
        line = infile.readline()
        if len(line) == 0:
            return [],[],[],[]
    while line[0] != '#':
            #k = int(line.split()[0])
            X.append(float(line.split()[1]))
            Y.append(float(line.split()[2]))
            Xr.append( X[len(X)-1]-float(line.split()[8]) )
            Yr.append( Y[len(Y)-1]-float(line.split()[9]) )
            line = infile.readline()
            if len(line) == 0:
                return [],[],[],[]
    return X,Y,Xr,Yr

def animate(data):
    # update the data
    xdata,ydata, xrdata, yrdata = grab_data()
    #xmin, xmax = ax.get_xlim()
    lineA.set_data(xdata, ydata)
    lineB.set_data(xrdata, yrdata)
    return lineA, lineB

def init():
    lineA.set_data([], [])
    lineB.set_data([], [])
    return lineA,lineB

parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
              help="Path to input file (output from vme-nmpc).", metavar="FILE")
parser.add_option("-i", "--interval", dest="interval",
              help="Graph refresh interval",)
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")
(options, args) = parser.parse_args()

if options.stdintrue :
    grab_data.infile = stdin
else:
    grab_data.infile = open(options.filename, 'r')

fig = plt.figure()
ax = fig.add_subplot(111)
lineA, = ax.plot([], [], 'o-', lw=2)
lineB, = ax.plot([], [], 'o-', lw=2)
ax.set_ylim(-2.5, 2.5)
ax.set_xlim(-.5, 10)
ax.grid()
anim = animation.FuncAnimation(fig, animate, init_func=init, interval=float(options.interval), blit=True)
plt.show()
