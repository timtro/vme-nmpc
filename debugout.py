#! /usr/bin/python

from optparse import OptionParser
from sys import stdin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def grab_data():
    infile = grab_data.infile
    X,Y = [],[]
    line = infile.readline()
    while line[0] != '#':
        line = infile.readline()
        if len(testline) ==0:
            break # EOF
    while line[0] == '#':
        line = infile.readline()
        if len(testline) ==0:
            break # EOF
    while line[0] != '#':
            #k = int(line.split()[0])
            X.append(float(line.split()[1]))
            Y.append(float(line.split()[2]))
            line = infile.readline()
            if len(testline) ==0:
                break # EOF
    return X,Y

def animate(data):
    # update the data
    xdata,ydata = grab_data()
    #xmin, xmax = ax.get_xlim()
    line.set_data(xdata, ydata)
    return line,

def init():
    line.set_data([], [])
    return line,

parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
              help="Path to input file (output from vme-nmpc).", metavar="FILE")
parser.add_option("-s", "--stdin",
              action="store_true", dest="stdintrue", default=False,
              help="Take input from stdin. vme-nmpc [otps] | thisscript.py -s")
(options, args) = parser.parse_args()

print(options.stdintrue)

if options.stdintrue :
    grab_data.infile = stdin
else:
    grab_data.infile = open(options.filename, 'r')

fig = plt.figure()
ax = fig.add_subplot(111)
line, = ax.plot([], [], 'o-', lw=2)
ax.set_ylim(0, .3)
ax.set_xlim(0, .3)
ax.grid()
anim = animation.FuncAnimation(fig, animate, init_func=init, interval=100, blit=True)
plt.show()
