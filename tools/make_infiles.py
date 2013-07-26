#!/usr/bin/env python
"""
 * make_infiles.py
 * Author : Timothy A.V. Teatro
 * Date   : 2013-06-10
 *
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013 - Timothy A.V. Teatro
 *
 * vme-nmpc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License,or
 * (at your option) any later version.
 *
 * vme-nmpc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with vme-nmpc. If not,see <http://www.gnu.org/licenses/>.

Description
===========

This is written to generate vme-nmpc input files en masse by incrementally
combining sets of values for parameters.

"""

import numpy as np
import subprocess
from multiprocessing import Process, Queue
from Queue import Empty
import itertools as it

class infile_iterator():
    def __init__(self):
        self.Nr = np.arange(30, 120, 30)
        self.Cr = np.arange(3, 15, 3)
        self.Tr = np.arange(0.005, .205, 0.005)
        self.Rr = np.concatenate((np.arange(0, 1.25, .25),
                             np.arange(1, 15, 5), np.arange(25, 100, 25)))
        self.Qr = np.arange(0, 100, 25)
        self.Q0r = np.arange(0, 100, 25)
        self.ins = []
        self.im = {'N':0, 'C':1, 'T':2, 'R':3, 'Q':4, 'Q0':5}
        for N, C, T, R, Q, Q0 in it.product(self.Nr, self.Cr, self.Tr, self.Rr,
                                            self.Qr, self.Q0r):
            self.ins.append([N, C, T, R, Q, Q0])
        self.k = 0

    def pop(self):
        if (self.k >= len(self.ins)):
            return 1
        filename = 'sm_0713-0_' + '{:d}'.format(self.k).zfill(5)
        fdin = open(filename + '.in', 'w')
        fdin.write('N=' + '{:d}'.format(self.ins[self.k][self.im['N']]) + ';\n')
        fdin.write('C=' + '{:d}'.format(self.ins[self.k][self.im['C']]) + ';\n')
        fdin.write('T=' + '{:f}'.format(self.ins[self.k][self.im['T']]) + ';\n')
        fdin.write('eps = 0.08;\n')
        fdin.write('R=' + '{:f}'.format(self.ins[self.k][self.im['R']]) + ';\n')
        fdin.write('Q0=' + '{:f}'.format(self.ins[self.k][self.im['Q0']])
                + ',' + '{:d}'.format(self.ins[self.k][self.im['Q0']]) + ';\n')
        fdin.write('Q=' + '{:f}'.format(self.ins[self.k][self.im['Q']])
                + ',' + '{:d}'.format(self.ins[self.k][self.im['Q']]) + ';\n')
        fdin.write('m=5;\n')
        fdin.write('n=1;\n')
        fdin.write('dg=0.05;\n')
        fdin.write('cruising_speed=0.4;\n')
        fdin.write('tgttol=0.1;\n')
        fdin.write('tgt=10.00,0.00,0.00,-0.50;\n')
        fdin.write('S=0,0,0,0,0;\n')
        fdin.write('obst= 2.01193833342,-4.2560887012,\
1.15670182586,-2.96357211325,\
3.54042965579,-2.62589038265,\
2.63973719878,-3.93872321394,\
3.82229827323,-4.91759539923,\
6.56567139125,-3.32937337262,\
6.52475553204,-3.88977622715,\
6.88872089189,-2.20600930894,\
0.872173704782,-2.96846162671,\
2.56583304271,-3.01358573066,\
2.21809556124,-1.37664621717,\
6.3062900134,-1.31299337894,\
4.81270718895,-2.35359034401,\
7.20530895089,-2.24351989205,\
7.12161201577,-1.09581545984,\
8.8415761708,0.0857585946345,\
0.904669231242,-1.75326049254,\
4.44763419084,-0.00342567137573,\
3.1212989302,-1.26001631771,\
3.95184535187,-2.05375595627,\
5.18563288954,-1.10764193341,\
6.62071918441,0.260422061344,\
6.52528959439,-1.21546150933,\
7.3378542077,-1.6444479591,\
1.39963084819,0.641736706179,\
1.27553274518,0.59911590766,\
2.64864016112,-0.901805421495,\
3.67932498286,1.57457225239,\
3.138622903,1.1360269089,\
7.80066273551,-0.731339478696,\
6.09912682141,-0.410666703907,\
7.02785520705,-1.58673507407,\
1.79320564009,1.8809713441,\
1.65535085733,0.823165778084,\
3.73507342964,0.20560058768,\
2.90608090504,0.51831976772,\
5.41368635985,0.565339817102,\
7.39274823444,1.79197594161,\
7.34111445859,1.3581790948,\
7.46327058952,1.24587580795,\
-0.0475676922668,1.88481779528,\
3.31194155474,3.49035262682,\
1.92139872431,-0.484797460097,\
4.31637049406,2.59302810384,\
5.235111071,2.80912961681,\
6.84065262488,3.70590702714,\
6.53943396309,0.915272391018,\
6.87368719854,2.36298891584;\n')
        fdin.close()
        self.k += 1
        return filename

def prep_and_run(filename):
    print
    print
    print(filename)
    print
    print
    if filename == 1:
        exit(0)
    fdout = open(filename + '.out', 'w')
    rcode = subprocess.call([vme_nmpc, '-vf' + filename + '.in'], stdout=fdout)
    if (rcode == 0):
        subprocess.call([analyse, '-f' + filename + '.out',
                     '-p' + filename+ '.jpg', '-csummary.csv'])
    else:
        subprocess.call(['rm',filename + '.in'])
    subprocess.call(['rm', filename + '.out'])
    fdout.close()

def do_work(q):
    while True:
        try:
            filename = q.get(block=False)
            prep_and_run(filename)
        except Empty:
            break

vme_nmpc = '/home/timtro/workspace/vme-nmpc/build/vme-nmpc'
analyse = '/home/timtro/workspace/vme-nmpc/tools/analyse_output.py'

if __name__ == '__main__':
    infi = infile_iterator()
    work_queue = Queue()
    while 1:
        filename = infi.pop()
        if (filename == 1):
            print('Finished!\n')
            break
        work_queue.put(filename)
    processes = [Process(target=do_work, args=(work_queue,)) for i in range(8)]
    for p in processes:
        p.start()
    for p in processes:
        p.join()