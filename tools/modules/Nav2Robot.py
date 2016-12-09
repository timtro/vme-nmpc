"""
 * This file is part of vme-nmpc.
 *
 * Copyright (C) 2013--15 Timothy A.V. Teatro - All rights Reserved
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

This module contains a class for wrapping the connection and communication with
a TCP server accepting Nav2 server interface.

"""

import socket
import numpy as np
from math import sin, cos, degrees, radians, atan2
import time


class Nav2Robot:

    def __init__(self, addy, portno):
        '''
        Initialize with the address and port of the robot.
        '''
        self.hostname = addy
        self.hostport = portno

    def connect(self):
        '''
        The object was initialized with the address and port of the robot.
        Now, use them to connect and return the reuslt of socket.connect().
        '''
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        return self.socket.connect((self.hostname, self.hostport))

    def command(self, *arg):
        '''
        Taks an arbitrarily long list of arguments, of the form:
           cmd a1, a2, a3,...
        such that cmd is a string contaiing a turtle command and the remaining
        arguments are converted to strings (if nessisary) and concatinated with
        a single space between before a newline character is appended and the
        concatination is passed to send for transmission to the robot.

        The idea is to give this sort of interface:
          vme.command('v', 0, 0.5, .25)
        which would send the command
          v 0 0.5 0.25\n
        to the robot.
        '''
        cmd = arg[0]
        if len(arg) > 1:
            for item in arg[1:]:
                cmd += ' ' + str(item)
        cmd += '\n'
        return self.socket.send(cmd.encode())

    def sendline(self, cmd):
        '''
        Packs a newline character onto the end of cmd (if needed) and sends it
        over the socket, encoding() along the way.
        '''
        if cmd[-1] != '\n':
            cmd += '\n'
        return self.socket.send(cmd.encode())

    def stop(self):
        '''
        Stop the robot by sending it 's'.

        Not using self.command() to reduce opportunity for error,
        since stop is a rather critical function.
        '''
        return self.socket.send("s\n".encode())

    # s is the same as stop
    s = stop

    def locationa(self):
        self.sendline('q')
        return np.array(
            [float(str) for str in self.socket.makefile().readline().split()])

    def locationl(self):
        self.sendline('q')
        return [
            float(str) for str in self.socket.makefile().readline().split()
        ]

    def locationt(self):
        self.sendline('q')
        return tuple(
            [float(str) for str in self.socket.makefile().readline().split()])

    location = locationa

    def originate(self):
        return self.sendline('p 0 0 0')

    def p(self, x=None, y=None, theta=None):
        return self.command('p', x, y, theta)

    def rt(self, theta):
        return self.command('rt', theta)

    def lt(self, theta):
        return self.command('lt', theta)

    def mv(self, dist, theta):
        return self.command('mv', dist, theta)

    def fd(self, dist):
        return self.command('fd', dist)

    def bk(self, dist):
        return self.command('bk', dist)

    def av(self, vx, vy):
        return self.command('av', vx, vy)

    def v(self, theta, v, omega=None):
        return self.command('v', theta, v, omega)

    def o(self, theta):
        return self.command('o', theta)

    def avv(self, v, theta):
        rads = np.radians(theta)
        return self.command('av', cos(rads) * v, sin(rads) * v)

    def pointat(self, Xat):
        X = self.location()
        DX = Xat - X[:2]
        return self.o(degrees(atan2(DX[1], DX[0])))

    def displacement_from(self, Xat):
        X = self.location()
        return Xat[:2] - X[:2]

    def distance_to(self, Xat):
        return np.sqrt(np.sum(self.displacement_from(Xat)**2))

    def hold_for_locomotion(self, threshold=0.1):
        there = self.location()
        time.sleep(.5)  # Let the robot start moving.
        while True:
            time.sleep(0.25)
            here = self.location()
            if np.sum((there[:2] - here[:2])**2) <= threshold**2:
                break
            there = here

    def hold_for_rotation(self, threshold=0.5):
        there = self.location()
        time.sleep(.5)  # Let the robot start moving.
        while True:
            time.sleep(0.25)
            here = self.location()
            if np.absolute(there[2] - here[2]) <= threshold:
                break
            there = here
